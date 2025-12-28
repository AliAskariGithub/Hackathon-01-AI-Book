/**
 * ChatPanel - Main chat panel with message list, input, and controls.
 *
 * Renders the chat interface including:
 * - Header with title and close button
 * - Message list with auto-scroll
 * - Input field with submit button
 * - Clear chat button
 *
 * Per spec FR-006, FR-007, FR-008, FR-028.
 */

import React, { useRef, useEffect, useState, useCallback } from 'react';
import FocusTrap from 'focus-trap-react';
import { useChatContext } from './ChatContext';
import { useChatApi } from './useChatApi';
import ChatMessage from './ChatMessage';
import { X, Zap, BookOpen, Send, ArrowDown, Bot } from 'lucide-react';
import styles from './Chatbot.module.css';

export default function ChatPanel() {
  const {
    isPanelOpen,
    closePanel,
    messages,
    isLoading,
    addUserMessage,
    addAssistantMessage,
    updateAssistantMessage,
    setMessageError,
    setLoading,
    clearConversation,
    getConversationHistory,
    conversationId,
  } = useChatContext();

  const { sendMessage, retryMessage } = useChatApi();
  const [inputValue, setInputValue] = useState('');
  const [showScrollButton, setShowScrollButton] = useState(false);
  const messagesEndRef = useRef(null);
  const messageListRef = useRef(null);
  const inputRef = useRef(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages]);

  // Focus input when panel opens
  useEffect(() => {
    if (isPanelOpen && inputRef.current) {
      // Small delay to ensure panel is rendered
      const timer = setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
      return () => clearTimeout(timer);
    }
  }, [isPanelOpen]);

  // Handle keyboard escape
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.key === 'Escape' && isPanelOpen) {
        closePanel();
      }
    };
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isPanelOpen, closePanel]);

  // Track scroll position for scroll-to-bottom button
  useEffect(() => {
    const handleScroll = () => {
      if (messageListRef.current) {
        const { scrollTop, scrollHeight, clientHeight } = messageListRef.current;
        setShowScrollButton(scrollHeight - scrollTop - clientHeight > 100);
      }
    };

    const listEl = messageListRef.current;
    listEl?.addEventListener('scroll', handleScroll);
    return () => listEl?.removeEventListener('scroll', handleScroll);
  }, []);

  // Scroll to bottom handler
  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  // Submit message
  const handleSubmit = useCallback(
    async (e) => {
      e?.preventDefault();
      const query = inputValue.trim();
      if (!query || isLoading) return;

      // Clear input immediately
      setInputValue('');

      // Add user message to state
      addUserMessage(query);

      // Add placeholder assistant message
      const assistantMessageId = addAssistantMessage({
        content: '',
        isLoading: true,
        citations: [],
      });

      setLoading(true);

      try {
        const response = await sendMessage({
          query,
          conversationHistory: getConversationHistory(),
          conversationId,
        });

        if (response.error) {
          // Handle error response
          setMessageError(assistantMessageId, {
            type: response.error,
            message: getErrorMessage(response.error),
            retryAvailable: true,
          });
        } else {
          // Update with successful response
          updateAssistantMessage(assistantMessageId, {
            content: response.answer,
            citations: response.citations || [],
            isLoading: false,
          });
        }
      } catch (error) {
        // Handle network/unexpected errors
        setMessageError(assistantMessageId, {
          type: 'network',
          message: getErrorMessage('network'),
          retryAvailable: true,
        });
      } finally {
        setLoading(false);
      }
    },
    [
      inputValue,
      isLoading,
      addUserMessage,
      addAssistantMessage,
      updateAssistantMessage,
      setMessageError,
      setLoading,
      sendMessage,
      getConversationHistory,
      conversationId,
    ]
  );

  // Handle retry for failed messages
  const handleRetry = useCallback(
    async (messageId) => {
      // Find the user message before this assistant message
      const messageIndex = messages.findIndex((m) => m.id === messageId);
      if (messageIndex <= 0) return;

      const userMessage = messages[messageIndex - 1];
      if (userMessage.role !== 'user') return;

      // Clear error state and set loading
      updateAssistantMessage(messageId, {
        error: null,
        isLoading: true,
        content: '',
      });

      setLoading(true);

      try {
        // Get conversation history up to (but not including) the failed exchange
        const historyUpToFailure = messages
          .slice(0, messageIndex - 1)
          .filter((msg) => !msg.isLoading && !msg.error)
          .map((msg) => ({
            role: msg.role,
            content: msg.content,
          }));

        const response = await retryMessage({
          query: userMessage.content,
          conversationHistory: historyUpToFailure,
          conversationId,
        });

        if (response.error) {
          setMessageError(messageId, {
            type: response.error,
            message: getErrorMessage(response.error),
            retryAvailable: true,
          });
        } else {
          updateAssistantMessage(messageId, {
            content: response.answer,
            citations: response.citations || [],
            isLoading: false,
          });
        }
      } catch (error) {
        setMessageError(messageId, {
          type: 'network',
          message: getErrorMessage('network'),
          retryAvailable: true,
        });
      } finally {
        setLoading(false);
      }
    },
    [
      messages,
      updateAssistantMessage,
      setMessageError,
      setLoading,
      retryMessage,
      conversationId,
    ]
  );

  // Handle input change
  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  // Handle Enter key (submit) and Shift+Enter (newline)
  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  if (!isPanelOpen) {
    return null;
  }

  return (
    <FocusTrap
      focusTrapOptions={{
        allowOutsideClick: true,
        fallbackFocus: () => inputRef.current,
      }}
    >
      <div
        id="chat-panel"
        className={styles.panel}
        role="dialog"
        aria-label="Chat with book assistant"
        aria-modal="true"
      >
        {/* Header */}
        <header className={styles.header}>
          <div className={styles.headerLeft}>
            <div className={styles.headerAvatar}>
              <BookOpen size={20} strokeWidth={2} />
            </div>
            <div className={styles.headerTitle}>
              <h2 className={styles.title}>AI Assistant</h2>
              <p className={styles.subtitle}>Always here to help</p>
            </div>
          </div>
          <div className={styles.headerActions}>
            <button
              type="button"
              className={styles.clearButton}
              onClick={clearConversation}
              aria-label="Clear chat history"
              title="Clear chat"
            >
              <Zap size={18} strokeWidth={2} />
            </button>
            <button
              type="button"
              className={styles.closeButton}
              onClick={closePanel}
              aria-label="Close chat panel"
            >
              <X size={20} strokeWidth={2} />
            </button>
          </div>
        </header>

        {/* Message List */}
        <div
          ref={messageListRef}
          className={styles.messageList}
          role="log"
          aria-live="polite"
          aria-label="Chat messages"
        >
          {messages.length === 0 ? (
            <div className={styles.emptyState}>
              <div className={styles.emptyIcon}>
                <Bot size={32} strokeWidth={2} />
              </div>
              <p>Ask me anything about robotics and Isaac Sim!</p>
              <p className={styles.emptyHint}>
                Try: "What is URDF?" or "Explain forward kinematics"
              </p>
            </div>
          ) : (
            messages.map((message) => (
              <ChatMessage
                key={message.id}
                message={message}
                onRetry={() => handleRetry(message.id)}
              />
            ))
          )}
          <div ref={messagesEndRef} />
        </div>

        {/* Scroll to Bottom Button */}
        {showScrollButton && (
          <button
            className={styles.scrollButton}
            onClick={scrollToBottom}
            aria-label="Scroll to bottom"
            title="Scroll to bottom"
          >
            <ArrowDown size={20} strokeWidth={2} />
          </button>
        )}

        {/* Input Form */}
        <div className={styles.inputContainer}>
          <textarea
            ref={inputRef}
            className={styles.input}
            value={inputValue}
            onChange={handleInputChange}
            onKeyDown={handleKeyDown}
            placeholder="Type your question..."
            aria-label="Type your question"
            disabled={isLoading}
            rows={1}
            maxLength={2000}
          />
          <button
            type="button"
            onClick={handleSubmit}
            className={styles.submitButton}
            disabled={!inputValue.trim() || isLoading}
            aria-label="Send message"
          >
            {isLoading ? (
              <span className={styles.spinner} aria-hidden="true" />
            ) : (
              <Send size={20} strokeWidth={2} />
            )}
          </button>
        </div>
      </div>
    </FocusTrap>
  );
}

// Error message mapping per spec data-model.md
function getErrorMessage(errorType) {
  const messages = {
    retrieval: 'Could not search book content. Please try again.',
    generation: 'Could not generate response. Please try again.',
    rate_limit: 'Too many requests. Please wait 60 seconds.',
    validation: 'Invalid request. Please try again.',
    internal: 'Something went wrong. Please try again.',
    backend: 'Something went wrong. Please try again.',
    network: 'Unable to connect. Check your internet.',
    timeout: 'Request timed out. Please try again.',
  };
  return messages[errorType] || messages.internal;
}