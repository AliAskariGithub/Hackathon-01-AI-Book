/**
 * useChatApi - Custom hook for backend API communication.
 *
 * Handles:
 * - POST requests to /api/chat
 * - 30s timeout with AbortController
 * - Auto-retry once on timeout (T024)
 * - Cold start detection (>10s response time) (T024b)
 * - Error type classification
 *
 * Per spec FR-002, FR-014, FR-014b.
 */

import { useCallback, useRef, useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Request timeout in ms (30 seconds per spec)
const REQUEST_TIMEOUT = 30000;

// Cold start detection threshold in ms (10 seconds per spec FR-014b)
const COLD_START_THRESHOLD = 10000;

/**
 * Hook for chat API communication.
 *
 * @returns {Object} API methods: sendMessage, retryMessage, isColdStart
 */
export function useChatApi() {
  const { siteConfig } = useDocusaurusContext();
  const backendUrl = siteConfig.customFields?.backendUrl || 'https://aliaskariface-backend-hackathon-01.hf.space';

  // Track cold start state for UI feedback
  const [isColdStart, setIsColdStart] = useState(false);
  const coldStartTimerRef = useRef(null);

  /**
   * Send a chat message to the backend.
   *
   * @param {Object} params - Request parameters
   * @param {string} params.query - User's question
   * @param {Array} params.conversationHistory - Previous messages
   * @param {string} params.conversationId - Conversation tracking ID
   * @param {boolean} isRetry - Whether this is a retry attempt
   * @returns {Promise<Object>} ChatResponse or error
   */
  const sendMessage = useCallback(
    async ({ query, conversationHistory, conversationId }, isRetry = false) => {
      const startTime = Date.now();
      const controller = new AbortController();
      let timeoutId = null;

      // Start cold start timer (show message after 10s)
      coldStartTimerRef.current = setTimeout(() => {
        setIsColdStart(true);
      }, COLD_START_THRESHOLD);

      try {
        // Set up timeout
        timeoutId = setTimeout(() => {
          controller.abort();
        }, REQUEST_TIMEOUT);

        const response = await fetch(`${backendUrl}/api/chat`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            query,
            conversation_history: conversationHistory,
            conversation_id: conversationId,
          }),
          signal: controller.signal,
        });

        // Clear timers on response
        clearTimeout(timeoutId);
        clearTimeout(coldStartTimerRef.current);
        setIsColdStart(false);

        // Handle rate limiting
        if (response.status === 429) {
          return {
            answer: null,
            citations: [],
            conversation_id: conversationId,
            error: 'rate_limit',
          };
        }

        // Handle other HTTP errors
        if (!response.ok) {
          const errorType =
            response.status >= 500 ? 'backend' : 'network';
          return {
            answer: null,
            citations: [],
            conversation_id: conversationId,
            error: errorType,
          };
        }

        const data = await response.json();

        // Log response time for debugging
        const duration = Date.now() - startTime;
        if (duration > COLD_START_THRESHOLD) {
          console.log(`[ChatApi] Response took ${duration}ms (cold start detected)`);
        }

        return data;
      } catch (error) {
        // Clear timers
        if (timeoutId) clearTimeout(timeoutId);
        if (coldStartTimerRef.current) clearTimeout(coldStartTimerRef.current);
        setIsColdStart(false);

        // Handle abort (timeout)
        if (error.name === 'AbortError') {
          // Auto-retry once on timeout (per spec T024)
          if (!isRetry) {
            console.log('[ChatApi] Request timed out, retrying once...');
            return sendMessage({ query, conversationHistory, conversationId }, true);
          }

          return {
            answer: null,
            citations: [],
            conversation_id: conversationId,
            error: 'timeout',
          };
        }

        // Handle network errors
        if (error.name === 'TypeError' || error.message.includes('fetch')) {
          return {
            answer: null,
            citations: [],
            conversation_id: conversationId,
            error: 'network',
          };
        }

        // Generic error
        return {
          answer: null,
          citations: [],
          conversation_id: conversationId,
          error: 'backend',
        };
      }
    },
    [backendUrl]
  );

  /**
   * Retry a failed message.
   * Same as sendMessage but indicates it's a manual retry.
   */
  const retryMessage = useCallback(
    (params) => {
      // Manual retry starts fresh (not counted as auto-retry)
      return sendMessage(params, false);
    },
    [sendMessage]
  );

  return {
    sendMessage,
    retryMessage,
    isColdStart,
  };
}

export default useChatApi;