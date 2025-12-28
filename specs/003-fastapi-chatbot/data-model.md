# Data Model: FastAPI Backend with Docusaurus Chatbot

**Feature**: 003-fastapi-chatbot | **Date**: 2025-12-28 | **Updated**: 2025-12-28

## Overview

This feature has a **stateless backend** - all state is managed client-side. The data model defines:
1. API request/response contracts
2. Client-side state structures
3. Reused entities from Spec-2

## Backend Entities (API Layer)

### ChatRequest

Request body for POST /api/chat endpoint.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| query | string | Yes | User's question (1-2000 chars) |
| conversation_history | ChatMessage[] | No | Previous messages for context |
| conversation_id | UUID | No | Client-generated tracking ID |

**Validation Rules**:
- `query` must be non-empty and ≤2000 characters
- `conversation_history` may be empty array or omitted
- `conversation_id` is passed through unchanged (client-generated)

### ChatResponse

Response body from POST /api/chat endpoint.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| answer | string | Yes* | LLM-generated response |
| citations | Citation[] | Yes | Source references (may be empty) |
| conversation_id | UUID | Yes | Echo or generate if not provided |
| error | string | No | Error type if request failed |

*`answer` is null/empty when `error` is present

**Error Types**:
- `retrieval` - Vector search failed
- `generation` - LLM call failed
- `rate_limit` - Hugging Face rate limit hit
- `validation` - Invalid request format
- `internal` - Unexpected server error

### ChatMessage

Single message in conversation history (shared with client).

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| role | string | Yes | "user" or "assistant" |
| content | string | Yes | Message text |

**Note**: Timestamps are client-side only, not sent to backend.

### Citation

Reference to book content (simplified from Spec-2).

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| title | string | Yes | Source page title |
| url | string | Yes | Relative Docusaurus route |
| score | float | No | Relevance score (pass-through only, not displayed in MVP) |

**URL Format**: `/docs/module-X/page-name` (relative, not absolute)

**Note on score field**: The score is passed through from the RAG agent but is not displayed in the MVP frontend. It's included for potential future use (e.g., visual relevance indicators). Decision documented in plan.md.

## Client-Side Entities (React)

### ClientChatMessage

Extended message for UI rendering.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique ID (uuid) |
| role | string | "user" or "assistant" |
| content | string | Message text |
| timestamp | Date | When message was created |
| citations | Citation[] | Only for assistant messages |
| isLoading | boolean | True while awaiting response |
| error | ErrorState | Present if message failed |

### ConversationState

React Context state structure.

| Field | Type | Description |
|-------|------|-------------|
| messages | ClientChatMessage[] | All messages in conversation |
| conversationId | string | UUID for tracking |
| isOpen | boolean | Chat panel visibility |
| isLoading | boolean | Waiting for backend response |

**localStorage Key**: `chatbot_conversation`

---

## localStorage Schema

The client persists conversation state to localStorage for cross-session continuity.

**Key**: `chatbot_conversation`

**Schema (version 1.0)**:
```json
{
  "conversationId": "550e8400-e29b-41d4-a716-446655440000",
  "messages": [
    {
      "id": "msg-uuid-1",
      "role": "user",
      "content": "What is URDF?",
      "timestamp": "2025-12-28T10:30:00.000Z"
    },
    {
      "id": "msg-uuid-2",
      "role": "assistant",
      "content": "URDF (Unified Robot Description Format) is...",
      "timestamp": "2025-12-28T10:30:05.000Z",
      "citations": [
        {"title": "URDF Basics", "url": "/docs/module-2/urdf-basics"}
      ]
    }
  ],
  "isOpen": true,
  "timestamp": "2025-12-28T10:30:05.000Z",
  "version": "1.0"
}
```

**Schema Fields**:

| Field | Type | Description |
|-------|------|-------------|
| conversationId | UUID | Client-generated conversation tracking ID |
| messages | ClientChatMessage[] | Full conversation history |
| isOpen | boolean | Whether chat panel was open when saved |
| timestamp | ISO8601 | Last modification time |
| version | string | Schema version for migrations |

**Version Field Purpose**:
- Enables future schema migrations
- If stored version < current version, migration logic can transform data
- Prevents data loss when schema changes

**Storage Limits**:
- localStorage typically 5-10MB per domain
- Single conversation unlikely to exceed limit
- Consider truncating very long conversations (>100 messages)

---

### ErrorState

Error information for UI display.

| Field | Type | Description |
|-------|------|-------------|
| type | string | Error category |
| message | string | User-friendly message |
| retryAvailable | boolean | Can user retry? |

**Error Type Mapping**:
| Backend Error | User Message | Retry? |
|---------------|--------------|--------|
| retrieval | "Could not search book content. Please try again." | Yes |
| generation | "Could not generate response. Please try again." | Yes |
| rate_limit | "Too many requests. Please wait 60 seconds." | Yes (delayed) |
| validation | "Invalid request. Please try again." | Yes |
| internal | "Something went wrong. Please try again." | Yes |
| (network) | "Unable to connect. Check your internet." | Yes |
| (timeout) | "Request timed out. Please try again." | Yes |

## Reused from Spec-2

The backend reuses these entities from `fullstack/backend/models.py`:

- **AgentConfig** - LLM configuration (model, temperature, etc.)
- **AgentState** - Runtime state with conversation
- **Conversation** - Message list with token tracking
- **Message** - Single conversation turn
- **RetrievalContext** - Retrieved chunks
- **RetrievalResult** - Single search result

## Entity Relationships

```
┌─────────────────────────────────────────────────────────────┐
│                         CLIENT                               │
├─────────────────────────────────────────────────────────────┤
│  ConversationState                                          │
│  ├── messages: ClientChatMessage[]                          │
│  │   ├── id, role, content, timestamp                       │
│  │   ├── citations: Citation[] (assistant only)             │
│  │   └── error: ErrorState (if failed)                      │
│  ├── conversationId: UUID                                   │
│  ├── isOpen: boolean                                        │
│  └── isLoading: boolean                                     │
│                                                             │
│  Persisted to localStorage as JSON                          │
└─────────────────────────────────────────────────────────────┘
                            │
                            │ POST /api/chat
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                         BACKEND                              │
├─────────────────────────────────────────────────────────────┤
│  ChatRequest                                                │
│  ├── query: string                                          │
│  ├── conversation_history: ChatMessage[]                    │
│  └── conversation_id: UUID (optional)                       │
│                                                             │
│              │                                              │
│              ▼                                              │
│  ┌─────────────────────────────┐                            │
│  │   Spec-2 Agent Integration  │                            │
│  │   - create_agent()          │                            │
│  │   - process_query()         │                            │
│  │   - AgentState (per-request)│                            │
│  └─────────────────────────────┘                            │
│              │                                              │
│              ▼                                              │
│  ChatResponse                                               │
│  ├── answer: string                                         │
│  ├── citations: Citation[] (URLs transformed)               │
│  ├── conversation_id: UUID                                  │
│  └── error: string (if failed)                              │
└─────────────────────────────────────────────────────────────┘
```

## State Flow

1. **User types question** → Add `ClientChatMessage` with `isLoading: false`
2. **Submit to backend** → Set `isLoading: true`, add placeholder assistant message
3. **Backend processes** → Create fresh `AgentState`, replay history, call `process_query`
4. **Response received** → Update assistant message with answer/citations
5. **Error occurred** → Update assistant message with `ErrorState`
6. **Persist** → Save `ConversationState` to localStorage

## Validation Summary

| Entity | Field | Rule |
|--------|-------|------|
| ChatRequest | query | 1-2000 chars, non-empty |
| ChatRequest | conversation_history | Valid ChatMessage array |
| ChatMessage | role | Must be "user" or "assistant" |
| ChatMessage | content | Non-empty string |
| Citation | url | Must be relative path starting with "/" |
