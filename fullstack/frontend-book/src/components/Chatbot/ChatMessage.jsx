/**
 * ChatMessage - Individual chat message component.
 *
 * Renders user and assistant message bubbles with:
 * - Different styling for user vs assistant
 * - Loading state with animated dots
 * - Error state with retry button
 * - Citations rendered as clickable links
 *
 * Per spec FR-007, FR-008, FR-009.
 */

import React from 'react';
import Link from '@docusaurus/Link';
import { User, RotateCw, Bot } from 'lucide-react';
import styles from './Chatbot.module.css';

export default function ChatMessage({ message, onRetry }) {
  const { role, content, citations, isLoading, error } = message;
  const isUser = role === 'user';
  const isAssistant = role === 'assistant';

  return (
    <div
      className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}
    >
      {/* Avatar */}
      <div className={styles.avatar} aria-hidden="true">
        {isUser ? (
          <User size={20} strokeWidth={2} />
        ) : (
          <Bot size={20} strokeWidth={2} />
        )}
      </div>

      {/* Message Content */}
      <div className={styles.messageContent}>
        {/* Loading State */}
        {isLoading && (
          <div className={styles.loadingMessage}>
            <div className={styles.loadingDots}>
              <span>AI is thinking</span>
              <div className={styles.dotContainer}>
                <span className={styles.dot}></span>
                <span className={styles.dot}></span>
                <span className={styles.dot}></span>
              </div>
            </div>
          </div>
        )}

        {/* Error State */}
        {error && !isLoading && (
          <div className={styles.errorMessage}>
            <p className={styles.errorText}>{error.message}</p>
            {error.retryAvailable && (
              <button
                type="button"
                className={styles.retryButton}
                onClick={onRetry}
                aria-label="Retry sending message"
              >
                <RotateCw size={14} strokeWidth={2} />
                Retry
              </button>
            )}
          </div>
        )}

        {/* Message Text */}
        {!isLoading && !error && content && (
          <>
            <div className={styles.messageText}>
              {/* Render markdown-like content with simple formatting */}
              {content.split('\n').map((paragraph, index) => (
                <p key={index}>{paragraph}</p>
              ))}
            </div>

            {/* Citations */}
            {isAssistant && citations && citations.length > 0 && (
              <div className={styles.citations}>
                <span className={styles.citationsLabel}>Sources:</span>
                <ul className={styles.citationsList}>
                  {citations.map((citation, index) => (
                    <li key={index} className={styles.citation}>
                      <Link
                        to={citation.url}
                        className={styles.citationLink}
                        title={citation.title}
                      >
                        {citation.title}
                      </Link>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </>
        )}
      </div>
    </div>
  );
}