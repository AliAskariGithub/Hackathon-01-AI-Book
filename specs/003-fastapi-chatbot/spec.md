# Feature Specification: FastAPI Backend with Docusaurus Chatbot

**Feature Branch**: `003-fastapi-chatbot`
**Created**: 2025-12-28
**Status**: Draft
**Input**: User description: "FastAPI backend with Docusaurus-integrated chatbot connecting RAG agent to book with embedded chat widget"

## Technical Context

This specification builds on an **existing codebase**:

| Component | Location | Status |
|-----------|----------|--------|
| Docusaurus Frontend | `fullstack/frontend-book/` | Existing (functional) |
| RAG Agent | `fullstack/backend/agent.py` | Existing (Spec-2) |
| Vector Search | `fullstack/backend/retrieval.py` | Existing (Spec-1) |
| **FastAPI Backend** | `fullstack/backend/app.py` | **NEW** (this spec) |
| **Chat Widget** | `fullstack/frontend-book/src/components/Chatbot/` | **NEW** (this spec) |

The implementation **adds** a REST API layer (`app.py`) and chat widget to the existing infrastructure. It does NOT rebuild or duplicate existing functionality.

## User Scenarios & Testing *(mandatory)*

*Note: All user stories assume the existing Docusaurus book at `fullstack/frontend-book/` is deployed and functional. The chat widget enhances the existing reader experience.*

### User Story 1 - Single Question via Chat Widget (Priority: P1)

A reader browsing the Isaac Sim robotics book wants to ask a question about the content they're reading. They click a floating chat button, type their question, and receive an answer with clickable citations linking directly to relevant book pages.

**Why this priority**: This is the core value proposition - enabling readers to get instant answers while reading the book. Without this, no other features matter.

**Independent Test**: Can be fully tested by opening the book website, clicking the chat button, typing "What is URDF?", and verifying a response appears with clickable citation links that navigate to book pages.

**Acceptance Scenarios**:

1. **Given** a reader is on any book page, **When** they click the chat button, **Then** a chat panel opens ready for input
2. **Given** the chat panel is open, **When** the reader types a question and submits, **Then** they see a "thinking" indicator while waiting
3. **Given** a question was submitted, **When** the backend responds, **Then** the answer appears with formatted citations as clickable links
4. **Given** a citation link in the response, **When** the reader clicks it, **Then** they navigate to that book page without page reload (SPA navigation)

---

### User Story 2 - Multi-turn Conversation (Priority: P1)

A reader asks a follow-up question like "Tell me more about that" and the chatbot understands the context from the previous exchange, providing a relevant continuation.

**Why this priority**: Context retention is essential for natural conversation flow - without it, every question requires full context repetition, making the chat frustrating to use.

**Independent Test**: Can be tested by asking "What is URDF?" then following with "How is it used in Isaac Sim?" and verifying the second response builds on the first without needing to re-explain URDF.

**Acceptance Scenarios**:

1. **Given** a conversation with previous messages, **When** the reader asks a follow-up question, **Then** the response acknowledges and builds on prior context
2. **Given** an ongoing conversation, **When** the reader navigates to a different book page, **Then** the conversation history persists in the chat panel
3. **Given** a conversation with history, **When** the reader clicks "Clear" or "New Chat", **Then** the conversation resets (client-side state cleared, no backend notification needed since backend is stateless)

---

### User Story 3 - Chat Persists Across Navigation (Priority: P2)

A reader is mid-conversation when they click a citation link or navigate to another page. The chat panel and conversation remain visible and intact.

**Why this priority**: This is critical for usability - losing context during normal navigation would be extremely frustrating and break the reading flow.

**Independent Test**: Can be tested by starting a conversation, clicking a book navigation link, and verifying the chat panel remains open with full conversation history visible.

**Acceptance Scenarios**:

1. **Given** an open chat panel with messages, **When** the reader clicks a book navigation link, **Then** the chat panel remains open with all messages preserved
2. **Given** a minimized chat button, **When** the reader navigates between pages, **Then** the button remains visible in the same position
3. **Given** the reader refreshes the page, **When** the page reloads, **Then** the chat panel state (open/closed) and messages are restored from localStorage

---

### User Story 4 - Dark/Light Mode Support (Priority: P2)

A reader using the book's dark mode expects the chat widget to match the current theme, switching automatically when they toggle the theme.

**Why this priority**: Visual consistency with the book is important for professional appearance and readability, but not blocking for core functionality.

**Independent Test**: Can be tested by toggling the Docusaurus theme switcher and verifying the chat widget colors update to match.

**Acceptance Scenarios**:

1. **Given** the book is in light mode, **When** the chat panel is open, **Then** it uses light theme colors matching the book
2. **Given** the book is in dark mode, **When** the reader toggles to light mode, **Then** the chat panel colors update immediately without needing to close/reopen
3. **Given** a system preference for dark mode, **When** the reader first visits the book, **Then** the chat widget respects the system preference

---

### User Story 5 - Out-of-Scope Query Handling (Priority: P3)

A reader asks a question unrelated to the book content (e.g., "What's the weather?"). The chatbot politely declines and suggests asking about book topics.

**Why this priority**: Graceful handling of edge cases improves user experience but isn't blocking for the primary use case.

**Independent Test**: Can be tested by asking "What's the capital of France?" and verifying a polite decline message appears.

**Acceptance Scenarios**:

1. **Given** a question outside the book's scope, **When** the reader submits it, **Then** they receive a polite message explaining the chatbot only answers book-related questions
2. **Given** an out-of-scope response, **When** the reader asks a valid question next, **Then** the chatbot responds normally with book content

---

### Edge Cases

- What happens when the backend server is unreachable? (Show friendly error message with retry button)
- How does the system handle very long questions exceeding token limits? (Truncate with user notification)
- What happens if a cited URL no longer exists in the book? (Display citation without link, or show "page moved" indicator)
- What happens during slow network conditions? (Show loading state, timeout after 30 seconds with auto-retry)
- What happens when Hugging Face rate limits are hit? (Display rate limit message, suggest waiting 60 seconds)
- What happens on cold start? (Show extended loading state, acceptable up to 60 seconds)

## Requirements *(mandatory)*

### Functional Requirements

**Backend (API Server)**

*Note: Backend API is implemented in `fullstack/backend/app.py`, importing from existing `agent.py` (Spec-2). No duplication of agent logic.*

- **FR-001**: System MUST expose a POST /api/chat endpoint that accepts user questions and conversation context

  **Request Format**:
  ```json
  {
    "query": "string",
    "conversation_history": [{"role": "user|assistant", "content": "string"}],
    "conversation_id": "optional-uuid"
  }
  ```

  **Response Format**:
  ```json
  {
    "answer": "string",
    "citations": [{"title": "string", "url": "string"}],
    "conversation_id": "uuid",
    "error": "optional-string"
  }
  ```

- **FR-001b**: System MUST accept JSON Content-Type and return JSON responses
- **FR-001c**: Backend MUST be stateless - all conversation history passed in request (no server-side storage)
- **FR-002**: System MUST process questions through the existing RAG agent (Spec-2) and return responses with citations
  - Implementation: `app.py` imports `process_query()` from `agent.py`
  - Mapping: `ChatRequest` → `agent.process_query()` → `ChatResponse`
  - Async handling: `app.py` MUST use `async def` endpoint if `process_query()` is async, or call synchronously if sync
  - Note: Verify `process_query()` signature in Spec-2 during implementation
- **FR-003**: System MUST accept CORS requests from allowed origins:
  - `http://localhost:3000` (Docusaurus dev server)
  - `http://localhost:8000` (alternative dev port)
  - `https://{username}.github.io` (GitHub Pages production)
  - `https://huggingface.co` (HF Space preview)
- **FR-003b**: Backend MUST reject requests from unlisted origins
- **FR-003c**: Backend reads allowed origins from `ALLOWED_ORIGINS` environment variable (comma-separated list)
  - Example: `ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000,https://username.github.io`
  - If `ALLOWED_ORIGINS` not set, default to `http://localhost:3000` only (dev safety)
- **FR-004**: System MUST return responses in the structured format defined in FR-001
- **FR-005**: System MUST handle errors gracefully and return user-friendly error messages with appropriate HTTP status codes
- **FR-005b**: System MUST return 429 status code when Hugging Face rate limits are hit

**Frontend (Chat Widget)**

*Note: Widget is ADDED to existing Docusaurus frontend in `fullstack/frontend-book/`. Existing book pages and navigation remain unchanged.*

- **FR-006**: Widget MUST display as a floating button in the bottom-right corner by default
- **FR-006b**: Widget MUST be implemented as a standalone React component in `fullstack/frontend-book/src/components/Chatbot/`
- **FR-006c**: Widget MUST be rendered via @theme/Root swizzling using `--wrap` method (preserves upstream updates)
  - Command: `npm run swizzle @docusaurus/theme-classic Root -- --wrap`
  - Creates: `fullstack/frontend-book/src/theme/Root.js`
  - Rationale: Wrap method maintains compatibility with Docusaurus updates vs eject which requires manual maintenance
- **FR-007**: Widget MUST expand into a chat panel when clicked
- **FR-008**: Widget MUST display conversation history with user questions and assistant responses
- **FR-009**: Widget MUST render citations as clickable links that navigate within the book
- **FR-009b**: Citations MUST use relative Docusaurus URLs (e.g., /docs/urdf-basics) not absolute URLs
- **FR-009c**: Backend MUST transform full page URLs from Qdrant to Docusaurus route format using `BOOK_BASE_URL` environment variable
  - Example: Given `BOOK_BASE_URL=https://username.github.io/repo`, transform `https://username.github.io/repo/docs/urdf-basics` → `/docs/urdf-basics`
  - Uses URL parsing to strip base URL prefix
- **FR-010**: Widget MUST persist conversation state using React Context during SPA navigation
- **FR-010b**: Widget MUST persist full conversation state to localStorage with schema:
  ```json
  {
    "version": "1.0",
    "conversationId": "uuid",
    "messages": [{"role": "user|assistant", "content": "string", "timestamp": "ISO8601", "citations": []}],
    "timestamp": "ISO8601",
    "isPanelOpen": boolean
  }
  ```
  - UI state (panel open) persists to avoid reopening on every page load
  - Version field enables future schema migrations
- **FR-011**: Widget MUST support Docusaurus dark/light theme modes using theme CSS variables
- **FR-012**: Widget MUST provide a "Clear conversation" action that resets client-side state
- **FR-013**: Widget MUST show loading state while waiting for responses
- **FR-014**: Widget MUST display appropriate error messages for each error type:
  - Network errors: "Unable to connect. Check your internet connection."
  - Backend errors: "Something went wrong. Please try again."
  - Timeout errors: "Request timed out. Please try again."
  - Rate limit errors: "Too many requests. Please wait 60 seconds."
  - Cold start errors: "Server starting, this may take up to 60 seconds..."
- **FR-014b**: Widget MUST display cold start message when request takes >10 seconds
  - Detection: If request exceeds 10 seconds without response, assume cold start and update loading message
  - Message: "Server starting, this may take up to 60 seconds..."
- **FR-015**: Widget MUST display rate limit message and suggest waiting when 429 status received
- **FR-016**: Frontend MUST read backend URL from docusaurus.config.js customFields.backendUrl (loaded at build time)
  ```javascript
  customFields: {
    backendUrl: process.env.BACKEND_URL || 'http://localhost:8000'
  }
  ```
  - Uses `process.env.BACKEND_URL` during build, not runtime
  - Example .env: `BACKEND_URL=https://username-isaac-sim-chatbot-api.hf.space`
- **FR-017**: Widget MUST display as full-screen modal on screens narrower than 768px
- **FR-018**: Widget MUST provide manual retry button on error responses
- **FR-018b**: Widget MUST auto-retry once immediately after 30-second timeout expires, then show manual retry button if second attempt fails
  - Total wait time before manual retry: up to 60 seconds (30s + 30s)

**Accessibility**

- **FR-019**: Widget MUST be keyboard navigable (Tab to focus, Enter to submit, Escape to close)
- **FR-020**: Widget MUST include ARIA labels for screen readers on all interactive elements
- **FR-021**: Chat input MUST have focus trap when panel is open

**Deployment**

- **FR-022**: Backend MUST be deployable as a Hugging Face Space using FastAPI
- **FR-023**: Backend MUST include requirements.txt for Hugging Face Spaces deployment
- **FR-024**: Backend MUST include app.py as entry point for Hugging Face Spaces
- **FR-025**: Widget MUST be bundled with Docusaurus production build
- **FR-026**: Backend MUST validate all required environment variables on startup and fail fast with clear error messages if any are missing
  - Required vars: `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `GROQ_API_KEY`, `BOOK_BASE_URL`
  - Optional vars: `ALLOWED_ORIGINS` (defaults to localhost:3000)

### State Management Strategy

- Backend is stateless - no conversation storage on server
- Client maintains full conversation history in React Context
- Client persists conversation to localStorage for cross-session continuity
- Conversation ID is client-generated UUID for tracking purposes only

### Key Entities

*Note: These are NEW entities for the chat feature. Existing Spec-2 entities (AgentState, Message, RetrievalResult) are reused internally by `agent.py`.*

- **ChatMessage**: Single message in conversation
  - `role`: 'user' | 'assistant'
  - `content`: string
  - `timestamp`: ISO8601 date
  - `citations`: Citation[] (for assistant messages only, empty array for user messages)
- **Citation**: Reference to book content
  - `title`: string
  - `url`: relative Docusaurus route (e.g., `/docs/urdf-basics`)
  - `score`: float (pass-through from Spec-2, NOT displayed to users in MVP)
- **ChatRequest**: Incoming API request (query: string, conversation_history: ChatMessage[], conversation_id: optional UUID)
- **ChatResponse**: Backend API response (answer: string, citations: Citation[], conversation_id: UUID, error: optional string)
- **ConversationState**: Complete localStorage schema (see FR-010b)
  - `version`: '1.0' (for schema migrations)
  - `conversationId`: UUID
  - `messages`: ChatMessage[]
  - `timestamp`: ISO8601
  - `isPanelOpen`: boolean
- **ErrorState**: Error information
  - `type`: 'network' | 'backend' | 'timeout' | 'rate_limit' | 'cold_start'
  - `message`: string
  - `retry_available`: boolean

## Success Criteria *(mandatory)*

### Measurable Outcomes

*Note: Success criteria are measured on the existing book deployment with the new chat widget added.*

- **SC-001**: Users receive responses within 15 seconds on warm starts
- **SC-001b**: 99% of warm-start requests complete within 15 seconds
- **SC-001c**: Cold start responses within 60 seconds are acceptable (documented in UI)
- **SC-002**: Chat widget loads and becomes interactive within 2 seconds of page load
- **SC-003**: 95% of citation links correctly navigate to valid book pages (validated via manual E2E testing with 20+ test citations)
  - Note: Automated testing of citation navigation out of scope
- **SC-004**: Chat conversation persists across at least 10 consecutive page navigations without data loss
- **SC-005**: Widget correctly renders in both dark and light modes with no visual glitches
- **SC-006**: Users can complete a 5-message conversation without the widget losing state or crashing
- **SC-007**: Error states display user-friendly messages and recovery options in 100% of failure scenarios
- **SC-008**: Widget is fully usable via keyboard navigation only

## Assumptions

**Existing Infrastructure**
- Docusaurus book frontend already exists in `fullstack/frontend-book/` and is fully functional
- Spec-1 RAG Book Embeddings is complete - Qdrant vector database is populated with book content
- Spec-2 RAG Agent is fully implemented with `process_query()` function available in `fullstack/backend/agent.py`
- GROQ_API_KEY and other API keys are available for LLM inference

**Implementation Context**
- Backend API (`app.py`) MUST import and reuse existing `agent.py` - no duplication of agent logic
- Chat widget is an ADDITION to existing book, not building a new frontend from scratch
- Book is deployed on GitHub Pages with a known domain (e.g., username.github.io/repo)
- Docusaurus v2 or v3 with custom Root component support for global widget rendering
- Docusaurus supports swizzling @theme/Root for injecting global components

**Deployment**
- Hugging Face Space URL example: `https://huggingface.co/spaces/username/isaac-sim-chatbot-api`
  - Space name should follow pattern: `{project-name}-chatbot-api`
- Cold start latency of 30-60 seconds is acceptable on Hugging Face free tier
- Hugging Face free tier rate limits will be encountered during high usage
- Backend does not store conversation history - client is responsible for state
- Bottom-right positioning works for book layout (configurable positioning out of scope)

**Environment Variables**
- `BOOK_BASE_URL` environment variable configured with deployed book's base URL (e.g., `https://aliaskarigithub.github.io`)
- `ALLOWED_ORIGINS` configured with comma-separated allowed CORS origins

## Dependencies

**Existing Codebase (Must Be Complete - Validate Before Implementation)**

Pre-implementation validation commands:
```bash
cd fullstack/backend
python -c "from retrieval import search; print(search('test', top_k=1))"  # Spec-1
python -c "from agent import process_query; print('OK')"                   # Spec-2
```

- Spec-1: RAG Book Embeddings - `fullstack/backend/retrieval.py` with `search()` function
- Spec-2: RAG Agent - `fullstack/backend/agent.py` with `process_query()` function
- Existing frontend - `fullstack/frontend-book/` with Docusaurus configuration

**External Services**
- Hugging Face Space with FastAPI support
- Qdrant Cloud (populated by Spec-1)

**Runtime Dependencies**
- Docusaurus >= 2.0 (tested and compatible with v3.x)
  - Note: Swizzling method may differ slightly between major versions
- FastAPI >= 0.104.0
- Uvicorn >= 0.24.0
- focus-trap-react >= 10.0.0 (for accessibility)

**Environment Variables**
- `COHERE_API_KEY` - Cohere API key for embeddings
- `QDRANT_URL` - Qdrant Cloud cluster URL
- `QDRANT_API_KEY` - Qdrant Cloud API key
- `GROQ_API_KEY` - OpenRouter API key for LLM
- `BOOK_BASE_URL` - Deployed book base URL for citation transformation
- `ALLOWED_ORIGINS` - (optional) Comma-separated CORS origins

## Out of Scope

- User authentication or session management
- Server-side rate limiting or usage quotas (rely on Hugging Face limits)
- Multi-user chat rooms or collaboration
- Admin dashboard or analytics UI
- Payment integration
- Mobile native apps
- Real-time streaming (request/response is acceptable)
- Configurable widget positioning (fixed bottom-right)
- Building a new Docusaurus frontend (existing `fullstack/frontend-book/` is used)
- Modifying existing book pages or navigation (only adding chat widget overlay)
- Rewriting or duplicating RAG agent logic (reuse existing `agent.py`)
- Displaying citation relevance scores to users (scores passed through but not rendered)
- Automated E2E testing (manual testing checklist used instead)
