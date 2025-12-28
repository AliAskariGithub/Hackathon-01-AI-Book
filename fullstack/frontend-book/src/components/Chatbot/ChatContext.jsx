/**
 * ChatContext - React Context for chat state management with localStorage persistence.
 *
 * Manages conversation state across the Docusaurus site, persisting to localStorage
 * for cross-session continuity per spec FR-010.
 *
 * localStorage Schema (version 1.0):
 * {
 *   "conversationId": "uuid",
 *   "messages": [{"id": "uuid", "role": "user|assistant", "content": "string", "timestamp": "ISO8601", "citations": []}],
 *   "isPanelOpen": false,
 *   "timestamp": "ISO8601",
 *   "version": "1.0"
 * }
 */

import React, { createContext, useContext, useReducer, useEffect, useCallback } from 'react';

// Storage key and schema version
const STORAGE_KEY = 'chatbot_conversation';
const SCHEMA_VERSION = '1.0';

// Generate UUID v4
function generateUUID() {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

// Initial state
const initialState = {
  conversationId: generateUUID(),
  messages: [],
  isPanelOpen: false,
  isLoading: false,
  timestamp: new Date().toISOString(),
  version: SCHEMA_VERSION,
};

// Action types
const ActionTypes = {
  SET_PANEL_OPEN: 'SET_PANEL_OPEN',
  ADD_USER_MESSAGE: 'ADD_USER_MESSAGE',
  ADD_ASSISTANT_MESSAGE: 'ADD_ASSISTANT_MESSAGE',
  UPDATE_ASSISTANT_MESSAGE: 'UPDATE_ASSISTANT_MESSAGE',
  SET_MESSAGE_ERROR: 'SET_MESSAGE_ERROR',
  SET_LOADING: 'SET_LOADING',
  CLEAR_CONVERSATION: 'CLEAR_CONVERSATION',
  LOAD_FROM_STORAGE: 'LOAD_FROM_STORAGE',
};

// Reducer
function chatReducer(state, action) {
  const timestamp = new Date().toISOString();

  switch (action.type) {
    case ActionTypes.SET_PANEL_OPEN:
      return {
        ...state,
        isPanelOpen: action.payload,
        timestamp,
      };

    case ActionTypes.ADD_USER_MESSAGE:
      return {
        ...state,
        messages: [
          ...state.messages,
          {
            id: generateUUID(),
            role: 'user',
            content: action.payload,
            timestamp,
            citations: [],
          },
        ],
        timestamp,
      };

    case ActionTypes.ADD_ASSISTANT_MESSAGE:
      return {
        ...state,
        messages: [
          ...state.messages,
          {
            id: action.payload.id || generateUUID(),
            role: 'assistant',
            content: action.payload.content || '',
            timestamp,
            citations: action.payload.citations || [],
            isLoading: action.payload.isLoading || false,
            error: action.payload.error || null,
          },
        ],
        timestamp,
      };

    case ActionTypes.UPDATE_ASSISTANT_MESSAGE: {
      const { messageId, updates } = action.payload;
      return {
        ...state,
        messages: state.messages.map((msg) =>
          msg.id === messageId
            ? { ...msg, ...updates, timestamp }
            : msg
        ),
        timestamp,
      };
    }

    case ActionTypes.SET_MESSAGE_ERROR: {
      const { messageId, error } = action.payload;
      return {
        ...state,
        messages: state.messages.map((msg) =>
          msg.id === messageId
            ? { ...msg, error, isLoading: false, timestamp }
            : msg
        ),
        timestamp,
      };
    }

    case ActionTypes.SET_LOADING:
      return {
        ...state,
        isLoading: action.payload,
      };

    case ActionTypes.CLEAR_CONVERSATION:
      return {
        ...initialState,
        conversationId: generateUUID(),
        isPanelOpen: state.isPanelOpen, // Keep panel open state
        timestamp,
      };

    case ActionTypes.LOAD_FROM_STORAGE:
      return {
        ...action.payload,
        isLoading: false, // Never persist loading state
      };

    default:
      return state;
  }
}

// Context
const ChatContext = createContext(null);

// Custom hook for using chat context
export function useChatContext() {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within a ChatProvider');
  }
  return context;
}

// Load state from localStorage
function loadFromStorage() {
  if (typeof window === 'undefined') {
    return null;
  }

  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) {
      return null;
    }

    const parsed = JSON.parse(stored);

    // Schema version check
    if (parsed.version !== SCHEMA_VERSION) {
      console.warn(
        `Chat history cleared due to schema update (${parsed.version} -> ${SCHEMA_VERSION})`
      );
      localStorage.removeItem(STORAGE_KEY);
      return null;
    }

    return parsed;
  } catch (e) {
    console.error('Failed to load chat state from localStorage:', e);
    localStorage.removeItem(STORAGE_KEY);
    return null;
  }
}

// Save state to localStorage
function saveToStorage(state) {
  if (typeof window === 'undefined') {
    return;
  }

  try {
    // Don't persist loading state or transient error states
    const persistState = {
      conversationId: state.conversationId,
      messages: state.messages.map((msg) => ({
        id: msg.id,
        role: msg.role,
        content: msg.content,
        timestamp: msg.timestamp,
        citations: msg.citations || [],
        // Persist error for retry UI, but not isLoading
        error: msg.error || null,
      })),
      isPanelOpen: state.isPanelOpen,
      timestamp: state.timestamp,
      version: SCHEMA_VERSION,
    };

    localStorage.setItem(STORAGE_KEY, JSON.stringify(persistState));
  } catch (e) {
    console.error('Failed to save chat state to localStorage:', e);
  }
}

// Provider component
export function ChatProvider({ children }) {
  const [state, dispatch] = useReducer(chatReducer, initialState);

  // Load from localStorage on mount
  useEffect(() => {
    const stored = loadFromStorage();
    if (stored) {
      dispatch({ type: ActionTypes.LOAD_FROM_STORAGE, payload: stored });
    }
  }, []);

  // Save to localStorage on state changes
  useEffect(() => {
    saveToStorage(state);
  }, [state]);

  // Action creators
  const togglePanel = useCallback(() => {
    dispatch({ type: ActionTypes.SET_PANEL_OPEN, payload: !state.isPanelOpen });
  }, [state.isPanelOpen]);

  const openPanel = useCallback(() => {
    dispatch({ type: ActionTypes.SET_PANEL_OPEN, payload: true });
  }, []);

  const closePanel = useCallback(() => {
    dispatch({ type: ActionTypes.SET_PANEL_OPEN, payload: false });
  }, []);

  const addUserMessage = useCallback((content) => {
    dispatch({ type: ActionTypes.ADD_USER_MESSAGE, payload: content });
  }, []);

  const addAssistantMessage = useCallback((message) => {
    const id = generateUUID();
    dispatch({
      type: ActionTypes.ADD_ASSISTANT_MESSAGE,
      payload: { ...message, id },
    });
    return id;
  }, []);

  const updateAssistantMessage = useCallback((messageId, updates) => {
    dispatch({
      type: ActionTypes.UPDATE_ASSISTANT_MESSAGE,
      payload: { messageId, updates },
    });
  }, []);

  const setMessageError = useCallback((messageId, error) => {
    dispatch({
      type: ActionTypes.SET_MESSAGE_ERROR,
      payload: { messageId, error },
    });
  }, []);

  const setLoading = useCallback((isLoading) => {
    dispatch({ type: ActionTypes.SET_LOADING, payload: isLoading });
  }, []);

  const clearConversation = useCallback(() => {
    dispatch({ type: ActionTypes.CLEAR_CONVERSATION });
  }, []);

  // Get conversation history for API requests (excludes client-only fields)
  const getConversationHistory = useCallback(() => {
    return state.messages
      .filter((msg) => !msg.isLoading && !msg.error)
      .map((msg) => ({
        role: msg.role,
        content: msg.content,
      }));
  }, [state.messages]);

  const value = {
    // State
    state,
    conversationId: state.conversationId,
    messages: state.messages,
    isPanelOpen: state.isPanelOpen,
    isLoading: state.isLoading,

    // Actions
    togglePanel,
    openPanel,
    closePanel,
    addUserMessage,
    addAssistantMessage,
    updateAssistantMessage,
    setMessageError,
    setLoading,
    clearConversation,
    getConversationHistory,
  };

  return <ChatContext.Provider value={value}>{children}</ChatContext.Provider>;
}

export default ChatContext;