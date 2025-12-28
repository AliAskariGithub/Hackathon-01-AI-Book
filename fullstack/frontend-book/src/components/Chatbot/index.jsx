/**
 * Chatbot - Main chatbot component with floating action button.
 *
 * Renders a floating button that opens the chat panel overlay.
 * Integrates with ChatContext for state management.
 *
 * Per spec FR-005: Floating chat button in bottom-right corner.
 */

import React, { useState, useEffect } from 'react';
import { useChatContext } from './ChatContext';
import ChatPanel from './ChatPanel';
import { Bot, Sparkles } from 'lucide-react';
import styles from './Chatbot.module.css';

export default function Chatbot() {
  const { isPanelOpen, togglePanel, messages } = useChatContext();
  const [isHovered, setIsHovered] = useState(false);
  const [shouldPulse, setShouldPulse] = useState(true);

  // Count unread messages (assistant messages since last close)
  const hasUnreadMessages = messages.some(
    (msg) => msg.role === 'assistant' && !msg.error
  );

  const unreadCount = messages.filter(
    (msg) => msg.role === 'assistant' && !msg.error
  ).length;

  // Stop pulse animation after 5 seconds
  useEffect(() => {
    const timer = setTimeout(() => {
      setShouldPulse(false);
    }, 5000);
    return () => clearTimeout(timer);
  }, []);

  return (
    <>
      {/* Floating Action Button */}
      <button
        type="button"
        className={`
          ${styles.floatingButton} 
          ${isPanelOpen ? styles.hidden : ''} 
          ${shouldPulse ? styles.pulse : ''}
        `}
        onClick={togglePanel}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
        aria-label="Open chat assistant"
        aria-expanded={isPanelOpen}
        aria-controls="chat-panel"
      >
        {/* Glow effect */}
        <div className={styles.glow} aria-hidden="true" />

        {/* Button icon with animation */}
        <span className={styles.buttonIcon} aria-hidden="true">
          <Bot 
            size={28} 
            strokeWidth={2.5}
            style={{
              transform: isHovered ? 'rotate(12deg) scale(1.1)' : 'none',
              transition: 'transform 0.3s ease',
            }}
          />
        </span>

        {/* Sparkle effect on hover */}
        {isHovered && (
          <span className={styles.sparkle} aria-hidden="true">
            <Sparkles size={12} className="text-yellow-300" />
          </span>
        )}

        {/* Notification badge for new messages */}
        {hasUnreadMessages && messages.length > 0 && (
          <span 
            className={styles.badge} 
            aria-label={`${unreadCount} new message${unreadCount > 1 ? 's' : ''} available`}
          >
            {unreadCount}
          </span>
        )}
      </button>

      {/* Chat Panel */}
      <ChatPanel />
    </>
  );
}