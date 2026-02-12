# Implementation Plan: FastAPI Backend with Docusaurus Chatbot

**Branch**: `003-fastapi-chatbot` | **Date**: 2025-12-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-fastapi-chatbot/spec.md`

## Summary

Implement a REST API backend (FastAPI) connecting the Spec-2 RAG agent to a Docusaurus-embedded chat widget. The backend exposes a stateless `/api/chat` endpoint that processes questions through the existing RAG pipeline and returns responses with citations. The frontend chat widget persists across page navigation via React Context and localStorage, supports dark/light themes, and is deployed as a floating button accessible from any book page.

## Technical Context

**Language/Version**: Python 3.11 (backend), JavaScript/React (frontend)
**Primary Dependencies**:
- Backend: FastAPI 0.104+, Uvicorn 0.24+, existing Spec-1/2 deps
- Frontend: React 18, Docusaurus 3.x, focus-trap-react@^10.0.0
**Storage**: Client-side only (localStorage for conversation persistence)
**Testing**: pytest (backend, >80% coverage for app.py), manual E2E testing (frontend)
**Target Platform**: Hugging Face Spaces (backend), GitHub Pages (frontend)
**Project Type**: Web application (fullstack/backend + fullstack/frontend-book)
**Performance Goals**: 15s warm-start response, 2s widget load
**Constraints**: 60s cold start acceptable, stateless backend, free-tier compatible
**Scale/Scope**: Single-user conversations, no authentication

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Spec-First Development | PASS | Comprehensive spec completed with 26 FRs |
| II. AI-Assisted Development | PASS | Claude Code used for all development |
| III. Technical Accuracy | PASS | Based on Docusaurus docs, FastAPI docs |
| IV. Reproducible Workflows | PASS | Clear setup in quickstart.md |
| V. Clean Architecture | PASS | No hardcoded secrets, env-based config |
| VI. Modular Code | PASS | Separate app.py, reuses agent.py |

**Technology Stack Compliance**:
- Book platform: Docusaurus ✓
- Deployment: GitHub Pages (frontend) + Hugging Face (backend) ✓
- Backend: FastAPI ✓
- Vector storage: Qdrant Cloud ✓

**Post-Design Re-check**: All gates still pass. Design maintains clean separation.

## Project Structure

### Documentation (this feature)

```text
specs/003-fastapi-chatbot/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 research findings
├── data-model.md        # Entity definitions (includes localStorage schema)
├── quickstart.md        # Setup guide
├── contracts/
│   └── chat-api.yaml    # OpenAPI specification (generated from FastAPI)
└── checklists/
    ├── requirements.md  # Specification checklist
    └── testing.md       # Manual testing checklist (Phase 8)
```

### Source Code (repository root)

```text
fullstack/
├── backend/
│   ├── app.py              # FastAPI entry point (NEW)
│   ├── api_models.py       # Pydantic request/response models (NEW)
│   ├── validate_config.py  # Startup configuration validator (NEW)
│   ├── agent.py            # RAG agent (Spec-2, existing)
│   ├── retrieval.py        # Vector search (Spec-1, existing)
│   ├── models.py           # Data models (existing)
│   ├── requirements.txt    # HF Spaces dependencies (NEW)
│   ├── Dockerfile          # HF Spaces container (CONDITIONAL - see Phase 0)
│   ├── README.md           # HF Space setup instructions (NEW)
│   └── test/
│       └── test_api.py     # API endpoint tests (NEW)
│
└── frontend-book/
    ├── src/
    │   ├── components/
    │   │   └── Chatbot/        # Chat widget (NEW)
    │   │       ├── index.jsx       # Main component
    │   │       ├── ChatPanel.jsx   # Expandable panel
    │   │       ├── ChatMessage.jsx # Message bubble
    │   │       ├── ChatContext.jsx # State management
    │   │       └── Chatbot.module.css
    │   └── theme/
    │       └── Root.js         # Swizzled with --wrap (NEW)
    ├── docusaurus.config.js    # Add customFields.backendUrl
    └── package.json            # Add focus-trap-react@^10.0.0
```

**Structure Decision**: Web application with existing `fullstack/backend` and `fullstack/frontend-book` directories. Backend adds `app.py` as new FastAPI entry point alongside existing Spec-1/2 modules. Frontend adds `Chatbot` component folder and swizzles `@theme/Root` using --wrap method.

---

## Pre-Implementation Gate

**GATE: Verify Spec-1 and Spec-2 are complete and functional**

```bash
# Verify Spec-1: Retrieval works
cd fullstack/backend && python -c "from retrieval import search; r=search('test', top_k=1); print(f'Spec-1 OK: {len(r)} results')"
# Expected output: Spec-1 OK: 1 results (or similar non-zero count)

# Verify Spec-2: Agent works
cd fullstack/backend && python -c "from agent import create_agent; a=create_agent(); print('Spec-2 OK: Agent created')"
# Expected output: Spec-2 OK: Agent created
```

**Requirement**: Both commands must succeed with expected output before proceeding to Phase 0.

---

## Implementation Phases

### Phase 0: Research

**Goal**: Validate technical assumptions before implementation

**GATE**: Phase 0 must complete before Phase 1

**Tasks**:
1. Research Docusaurus Root swizzling methods (`--eject` vs `--wrap`)
   - Document in research.md: pros/cons, upstream update compatibility
   - Decision needed: Which method to use
2. Research Hugging Face Spaces deployment options (Docker vs standard Python Space)
   - Test if standard Python Space suffices for FastAPI (no Docker needed)
   - Document findings and decision
3. Test FastAPI CORS configuration with multiple origins
   - Verify CORS works with localhost:3000, localhost:8000, github.io
   - Document working configuration
4. Research citation URL transformation patterns
   - Compare regex vs URL parsing approaches
   - Select and document approach with rationale
5. Verify focus-trap-react compatibility with Docusaurus React version
   - Check Docusaurus React version in package.json
   - Verify focus-trap-react@^10.0.0 compatibility
6. Update research.md with all findings
7. Update data-model.md with localStorage schema (see below)

**localStorage Schema** (document in data-model.md, reference spec FR-010b):
```json
{
  "conversationId": "uuid",
  "messages": [{"role": "user|assistant", "content": "string", "timestamp": "ISO8601", "citations": []}],
  "timestamp": "ISO8601",
  "version": "1.0",
  "isPanelOpen": false
}
```
- `isPanelOpen`: Persists UI state to avoid reopening on every page load
- `citations`: Array for assistant messages, empty for user messages

**Validation**: All research questions answered in research.md

---

### Phase 1: Backend API Development

**Goal**: Create FastAPI server exposing /api/chat endpoint

**Tasks**:
1. Create `api_models.py` with Pydantic schemas:
   - `ChatRequest` (query, conversation_history, conversation_id)
   - `ChatResponse` (answer, citations, conversation_id, error)
   - `ChatMessage` (role, content)
   - `Citation` (title, url, score - pass-through only, not displayed in MVP)
   - `ErrorResponse` (error_type: network|backend|timeout|rate_limit, message, details)
2. Create `app.py` with FastAPI initialization
3. Add CORS middleware with configurable origins:
   ```python
   ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "").split(",") or [
       "http://localhost:3000",
       "http://localhost:8000",
       "https://aliaskarigithub.github.io",
       "https://huggingface.co"
   ]
   ```
   *(example - replace `aliaskarigithub` with actual GitHub username)*
   Reference: FR-003
4. Add `BOOK_BASE_URL` environment variable for citation URL transformation
5. Implement `get_book_base_url()` helper to derive base URL from env var
6. Implement `POST /api/chat` endpoint integrating `process_query` from agent.py
   - Use `async def` if `process_query` is async, verify signature first (ref: spec FR-002)
   - Check agent.py: if `async def process_query`, use `await process_query()` in async endpoint
   - If sync, can use sync endpoint or run in thread pool
7. Implement URL transformation for citations using configurable base URL:
   - Transform `https://user.github.io/repo/docs/page` → `/docs/page`
   - Use URL parsing (per research findings)
8. Implement `GET /health` endpoint with dependency status
9. Add error handling with typed error responses (ErrorResponse model)
10. Generate OpenAPI spec: After all endpoints implemented (Tasks 6-9 complete), save `app.openapi()` output to `contracts/chat-api.yaml`
    - Depends on: Tasks 6-9 complete (all endpoints implemented)
    - This is the final step of Phase 1

**Validation**:
```bash
curl http://localhost:8000/health
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query":"What is URDF?"}'
```

---

### Phase 2: Hugging Face Deployment Setup

**Goal**: Prepare files for HF Spaces deployment

**Tasks**:
1. Research result check: Determine if Docker Space or standard Python Space (from Phase 0)
2. Create `requirements.txt` with all dependencies:
   ```
   fastapi>=0.104.0
   uvicorn>=0.24.0
   python-dotenv>=1.0
   cohere>=5.0
   qdrant-client>=1.7
   openai>=1.0.0
   tiktoken>=0.5.0
   tenacity>=8.0.0
   ```
3. Create `Dockerfile` ONLY if Docker Space required per Phase 0 research
   - Note: FastAPI typically runs on standard HF Python Spaces with requirements.txt only
4. Create `validate_config.py` for startup environment validation:
   ```python
   REQUIRED_VARS = [
       "COHERE_API_KEY",
       "QDRANT_URL",
       "QDRANT_API_KEY",
       "GROQ_API_KEY",
       "BOOK_BASE_URL"
   ]
   # Raise clear error if any missing
   ```
5. Update `.env.example` with all variables including:
   ```
   BACKEND_URL=https://huggingface.co/spaces/{username}/isaac-sim-chatbot-api
   BOOK_BASE_URL=https://aliaskarigithub.github.io
   ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000,https://aliaskarigithub.github.io
   ```
   *(placeholders - replace `{username}` with actual values before deployment)*
6. Create `README.md` for HF Space with setup instructions
   - Document HF Space naming convention: `{username}/isaac-sim-chatbot-api`
   - Note: Replace `{username}` placeholder with actual GitHub/HF username
7. Test locally with `uvicorn app:app --port 8000`
8. Update quickstart.md with HF Space setup steps

**Validation**: Local server responds to health check and chat requests with all env vars validated

---

### Phase 3: Frontend Chat Widget

**Goal**: Create React chat component with state management

**Tasks**:
1. Create `ChatContext.jsx` with React Context + localStorage:
   - Implement localStorage schema from Phase 0 (version: "1.0")
   - Include schema version for future migrations
   - On localStorage load: if version !== '1.0', clear old data and show notification 'Chat history cleared due to update'
   - Migration helper: Future versions increment (e.g., '1.1'), implement migration logic based on detected version
2. Create `Chatbot/index.jsx` with floating button (bottom-right)
3. Create `ChatPanel.jsx` with message list and input field
4. Create `ChatMessage.jsx` with user/assistant bubbles
5. Create `Chatbot.module.css` using Docusaurus theme variables:
   ```css
   .chatPanel {
     background: var(--ifm-background-color);
     color: var(--ifm-font-color-base);
     border: 1px solid var(--ifm-color-emphasis-300);
   }
   ```
6. Add `focus-trap-react@^10.0.0` to package.json:
   ```bash
   cd fullstack/frontend-book && npm install focus-trap-react@^10.0.0
   ```

**Validation**: Widget renders in Docusaurus dev mode (`npm start`)

---

### Phase 4: Docusaurus Integration

**Goal**: Integrate widget globally via Root swizzle

**Tasks**:
1. Swizzle `@theme/Root` using --wrap method:
   ```bash
   cd fullstack/frontend-book
   npm run swizzle @docusaurus/theme-classic Root -- --wrap
   ```
   - Rationale: Wrap preserves upstream updates, eject requires manual maintenance
   - Creates `src/theme/Root.js`
2. Edit `Root.js` to wrap app with `ChatProvider` and render `Chatbot`:
   ```jsx
   import React from 'react';
   import { ChatProvider } from '@site/src/components/Chatbot/ChatContext';
   import Chatbot from '@site/src/components/Chatbot';

   export default function Root({children}) {
     return (
       <ChatProvider>
         {children}
         <Chatbot />
       </ChatProvider>
     );
   }
   ```
3. Add `customFields.backendUrl` to `docusaurus.config.js`:
   ```javascript
   customFields: {
     backendUrl: process.env.BACKEND_URL || 'http://localhost:8000',
   },
   ```
4. Pass backend URL to ChatContext via useDocusaurusContext hook:
   ```jsx
   import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

   function ChatProvider({children}) {
     const {siteConfig} = useDocusaurusContext();
     const backendUrl = siteConfig.customFields.backendUrl;
     // Pass backendUrl to context value
   }
   ```

**Validation**: Widget visible on all book pages, persists across navigation

---

### Phase 5: API Communication

**Goal**: Connect widget to backend API

**Tasks**:
1. Implement `useChatApi` hook with fetch POST to /api/chat
2. Import `Link` from `@docusaurus/Link` for citation rendering:
   ```jsx
   import Link from '@docusaurus/Link';
   // Use Link component to ensure client-side navigation
   <Link to={citation.url}>{citation.title}</Link>
   ```
3. Add loading state during requests ("Thinking..." indicator)
4. Parse ChatResponse and render citations as Docusaurus Links
5. Implement 30s timeout using AbortController:
   ```javascript
   const controller = new AbortController();
   const timeoutId = setTimeout(() => controller.abort(), 30000);

   try {
     const response = await fetch(url, {
       signal: controller.signal,
       // ...
     });
   } catch (error) {
     if (error.name === 'AbortError') {
       // Timeout - auto-retry once
     }
   } finally {
     clearTimeout(timeoutId);
   }
   ```
6. Implement auto-retry immediately after first 30s timeout (no delay), then show manual retry button if second attempt fails
   - Total potential wait time: up to 60 seconds (30s + 30s) before manual retry required
   - Reference: spec FR-018b
7. Implement cold start detection: if request duration >10s, update loading message to 'Server starting, this may take up to 60 seconds...'
   - Reference: spec FR-014b
8. Add manual retry button for all error states
9. Handle error types: network, backend, timeout, rate_limit (429), cold_start

**Validation**: End-to-end chat flow works with error handling

---

### Phase 6: Mobile & Accessibility

**Goal**: Responsive design and keyboard support

**Tasks**:
1. Add CSS media query for <768px full-screen modal:
   ```css
   @media (max-width: 767px) {
     .chatPanel {
       position: fixed;
       top: 0;
       left: 0;
       right: 0;
       bottom: 0;
       border-radius: 0;
     }
   }
   ```
2. Implement keyboard navigation:
   - Tab: Navigate between elements
   - Enter: Submit message / activate button
   - Escape: Close chat panel
   - Ensure all buttons (Submit, Clear, Retry) are keyboard accessible via Tab navigation and Enter activation
3. Add ARIA labels to all interactive elements:
   ```jsx
   <button aria-label="Open chat assistant">💬</button>
   <div role="dialog" aria-label="Chat with book assistant">
   <input aria-label="Type your question" />
   <div role="log" aria-live="polite">{messages}</div>
   ```
4. Implement focus trap when panel is open using focus-trap-react

**Validation**: Works on mobile viewport (Chrome DevTools), fully navigable by keyboard only

---

### Phase 7: Production Build Verification

**Goal**: Verify widget works in production build

**Tasks**:
1. Run production build:
   ```bash
   cd fullstack/frontend-book && npm run build
   ```
2. Verify Chatbot component included in build output (check build/assets/)
3. Test production build locally:
   ```bash
   npm run serve
   ```
4. Verify widget works in production mode:
   - Opens/closes correctly
   - Persists across navigation
   - Theme switching works

**Validation**: Production build includes widget and functions correctly

---

### Phase 8: Testing & Validation

**Goal**: Comprehensive testing with documented results

**Note**: Frontend testing is manual E2E only (automated E2E out of scope)

**Tasks**:
1. Write pytest tests for API endpoints (target: >80% coverage for app.py):
   ```bash
   cd fullstack/backend
   pytest test/test_api.py --cov=app --cov-fail-under=80 --cov-report=term-missing -v
   ```
   - `--cov-fail-under=80`: Ensures build fails if coverage drops below target
2. Create manual testing checklist at `checklists/testing.md`:
   - [ ] Chat opens on button click
   - [ ] Message submits and response displays
   - [ ] Citations render as clickable links
   - [ ] Citation links navigate to correct pages
   - [ ] 5-message conversation flow works
   - [ ] Dark/light mode switching works
   - [ ] Conversation persists across 10+ page navigations
   - [ ] localStorage persistence across sessions
   - [ ] Error state: backend down
   - [ ] Error state: rate limit (429)
   - [ ] Error state: network timeout
   - [ ] Out-of-scope query handled gracefully (e.g., ask 'What's the weather today?')
   - [ ] Mobile viewport (< 768px)
   - [ ] Keyboard navigation only
3. Test 5-message conversation flow
4. Test dark/light mode switching
5. Test citation navigation to book pages
6. Test error states (backend down, rate limit, timeout)
7. Test localStorage persistence across browser sessions
8. Measure response times for 10 test queries:
   - Validate SC-001: Average warm-start <15s
   - Document cold-start time (acceptable up to 60s)
9. Document results in validation report section of testing.md

**Validation**: All success criteria from spec verified, >80% backend coverage

---

### Phase 9: Deployment

**Goal**: Deploy to production environments

**Tasks**:
1. Deploy backend to Hugging Face Space:
   - Create Space: `{username}/isaac-sim-chatbot-api`
   - Upload files: app.py, api_models.py, validate_config.py, agent.py, retrieval.py, models.py, requirements.txt
   - Configure secrets: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, GROQ_API_KEY, BOOK_BASE_URL
   - **Note**: Replace `{username}` placeholder with actual HF username before deployment
2. Verify HF Space health endpoint:
   ```bash
   curl https://{username}-isaac-sim-chatbot-api.hf.space/health
   ```
   *(replace `{username}` with actual value)*
3. Update `docusaurus.config.js` with production backend URL:
   ```javascript
   customFields: {
     backendUrl: 'https://{username}-isaac-sim-chatbot-api.hf.space',
   },
   ```
   *(replace `{username}` with actual value)*
4. Deploy frontend to GitHub Pages:
   ```bash
   cd fullstack/frontend-book
   npm run build
   # Deploy via GitHub Actions or manual push
   ```
5. Verify end-to-end production flow:
   - Open production book URL
   - Test chat with real question
   - Verify citation links work
6. Document deployment URLs in quickstart.md:
   - Backend: `https://{username}-isaac-sim-chatbot-api.hf.space`
   - Frontend: `https://aliaskarigithub.github.io`
   *(replace `{username}` placeholder with actual values)*

**Validation**: Full end-to-end production flow works

---

## Phase Dependencies

```
Pre-Implementation Gate
         │
         ▼
    Phase 0: Research
         │
         ▼
    Phase 1: Backend API ──────────────────┐
         │                                  │
         ▼                                  │
    Phase 2: HF Deployment Setup            │
         │                                  │
         ├──────────────────────────────────┤
         │                                  │
         ▼                                  ▼
    Phase 3: Frontend Widget ──► Phase 4: Docusaurus Integration
                                           │
                                           ▼
                                    Phase 5: API Communication
                                           │
                                           ▼
                                    Phase 6: Mobile & Accessibility
                                           │
                                           ▼
                                    Phase 7: Production Build
                                           │
                                           ▼
                                    Phase 8: Testing & Validation
                                           │
                                           ▼
                                    Phase 9: Deployment
```

---

## Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Backend state | Stateless | Simpler, no session management |
| State persistence | React Context + localStorage | SPA navigation + cross-session |
| Widget integration | @theme/Root swizzle (--wrap) | Preserves upstream updates |
| URL transformation | Server-side with URL parsing | Consistent, configurable via BOOK_BASE_URL |
| Theme support | CSS custom properties | Auto-updates on toggle |
| Error handling | Typed errors | User-friendly messages per type |
| HF deployment | Per Phase 0 research | Standard Python Space likely sufficient |
| Citation score | Pass-through only | Not displayed in MVP (simplicity) |
| localStorage version | "1.0" | Enables future schema migrations |

---

## Dependencies

**From Spec-1** (Qdrant populated):
- `retrieval.py` - search() function
- `models.py` - RetrievalResult

**From Spec-2** (RAG agent):
- `agent.py` - create_agent(), process_query()
- `models.py` - AgentState, Message, Citation

**External**:
- Python 3.11+ (for backend and HF Spaces compatibility)
- Hugging Face Space for backend hosting
- GitHub Pages for frontend hosting
- focus-trap-react@^10.0.0 for accessibility

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| HF cold start delay | UI shows extended loading, documented 60s acceptable |
| HF rate limits | 429 handling with user message, 60s wait suggestion |
| Citation URL mismatch | Backend transformation with configurable BOOK_BASE_URL |
| State loss on navigation | React Context at Root level, localStorage backup |
| Missing env vars | Startup validation with clear error messages |
| Schema changes | localStorage version field for migrations |
| Research findings differ from assumptions | Re-evaluate design decisions in affected phases, update tasks accordingly, document changes in research.md |

---

## Complexity Tracking

No constitution violations requiring justification. Design follows all principles:
- Single backend entry point (app.py)
- Single widget component folder
- No new external services beyond specified stack
- Minimal new dependencies (FastAPI, focus-trap-react@^10.0.0)
