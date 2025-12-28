/**
 * Root Theme Component Wrapper
 *
 * Wraps the entire Docusaurus site with ChatProvider and renders the Chatbot widget.
 * This follows the --wrap swizzle pattern for minimal intrusion.
 *
 * Per spec FR-010: Chat widget persists across page navigation.
 */

import React from 'react';
import { ChatProvider } from '@site/src/components/Chatbot/ChatContext';
import Chatbot from '@site/src/components/Chatbot';

export default function Root({ children }) {
  return (
    <ChatProvider>
      {children}
      <Chatbot />
    </ChatProvider>
  );
}
