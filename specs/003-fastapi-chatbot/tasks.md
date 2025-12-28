# Tasks: FastAPI Backend with Docusaurus Chatbot

**Branch**: `003-fastapi-chatbot` | **Date**: 2025-12-28 | **Plan**: [plan.md](./plan.md)

> **Note**: Phase numbers in this file align with plan.md: Phase 1 = Pre-Implementation Gates, Phase 2 = Research (plan.md Phase 0), Phase 3+ = Backend/Frontend implementation.

## User Stories Summary

| Story | Priority | Description |
|-------|----------|-------------|
| US1 | P1 | Single Question Answering - Core Q&A with citations |
| US2 | P1 | Multi-turn Conversation - Context retention |
| US3 | P2 | Chat Persists Across Navigation - SPA behavior |
| US4 | P2 | Dark/Light Mode Support - Theme integration |
| US5 | P3 | Out-of-Scope Query Handling - Graceful boundaries |

---

## Phase 1: Pre-Implementation Gates

- [x] T001 Validate Spec-1: `cd fullstack/backend && python -c "from retrieval import search; print(search('test', top_k=1))"` *(modules import OK, network required for full test)*
- [x] T002 Validate Spec-2: `cd fullstack/backend && python -c "from agent import create_agent; print('OK')"` *(modules import OK, network required for full test)*

---

## Phase 2: Research

- [x] T003a Research Docusaurus Root swizzling (`--eject` vs `--wrap`), document in `specs/003-fastapi-chatbot/research.md` *(R2: --wrap selected)*
- [x] T003b Research HF Spaces deployment (Docker vs standard Python Space), document decision *(R1, R10: Docker Space selected)*
- [x] T003c Test FastAPI CORS with localhost:3000, localhost:8000, github.io origins *(R5: documented)*
- [x] T003d Research citation URL transformation (regex vs URL parsing), select approach *(R4: URL parsing selected)*
- [x] T003e Verify focus-trap-react@^10.0.0 compatibility with Docusaurus React version *(R7: documented)*
- [x] T003f Create `specs/003-fastapi-chatbot/data-model.md` documenting all Pydantic models, React entities, and localStorage schema *(already exists)*

---

## Phase 3: Backend API (US1 Core)

- [x] T004 [P] Create `fullstack/backend/api_models.py` with Pydantic models:
  - ChatRequest (query, conversation_history, conversation_id)
  - ChatResponse (answer, citations, conversation_id, error)
  - ChatMessage (role, content, timestamp, citations)
  - Citation (title, url, score)
  - ErrorResponse with error_type enum: `network|backend|timeout|rate_limit` (matches spec FR-014)
- [x] T005 [P] Create `fullstack/backend/validate_config.py` checking: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, OPENROUTER_API_KEY, BOOK_BASE_URL
- [x] T006 Create `fullstack/backend/app.py` with FastAPI app, CORS middleware:
  ```python
  ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "").split(",") or [
      "http://localhost:3000", "http://localhost:8000",
      "https://aliaskarigithub.github.io", "https://huggingface.co"
  ]
  ```
  *(Note: `aliaskarigithub` is example - replace with actual GitHub username)*
- [x] T007 Add `transform_citation_url(url, base_url)` helper using URL parsing (per T003d), base_url from `BOOK_BASE_URL` env var
- [x] T008 Implement `POST /api/chat` endpoint integrating agent.process_query in `fullstack/backend/app.py`
  - Note: Check if `agent.process_query()` is async - if yes, use `async def` endpoint with `await` (ref: spec FR-002)
- [x] T009 Implement `GET /health` endpoint in `fullstack/backend/app.py`
- [ ] T009b Update `specs/003-fastapi-chatbot/quickstart.md` with backend setup instructions
- [x] T010 Add error handling returning typed ErrorResponse in `fullstack/backend/app.py`
- [ ] T010b Generate OpenAPI spec: save `app.openapi()` to `specs/003-fastapi-chatbot/contracts/chat-api.yaml` (after all endpoints implemented: T006-T010)
- [ ] T011 Test backend: `curl -X POST localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query":"What is URDF?"}'`

---

## Phase 4: HF Deployment Setup

- [x] T012 Create `fullstack/backend/requirements.txt` with:
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
- [x] T013 Update `fullstack/backend/.env.example` with:
  ```
  BOOK_BASE_URL=https://aliaskarigithub.github.io
  ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000,https://aliaskarigithub.github.io
  BACKEND_URL=https://huggingface.co/spaces/{username}/isaac-sim-chatbot-api
  ```
  *(Note: `{username}` is placeholder - replace with actual HF username before deployment)*
- [x] T013b [CONDITIONAL] Create `fullstack/backend/Dockerfile` ONLY if T003b research determines Docker Space required. Otherwise, document in research.md that standard Python Space with requirements.txt suffices *(research.md documents Docker Space selected)*
- [x] T014 Create `fullstack/backend/README.md` for HF Space setup instructions *(added to existing README)*

---

## Phase 5: Frontend Widget (US1)

- [x] T015 [US1] Create `fullstack/frontend-book/src/components/Chatbot/ChatContext.jsx` with React Context + localStorage:
  ```json
  {
    "conversationId": "uuid",
    "messages": [{"id": "uuid", "role": "user|assistant", "content": "string", "timestamp": "ISO8601", "citations": []}],
    "isPanelOpen": false,
    "timestamp": "ISO8601",
    "version": "1.0"
  }
  ```
  - `id` field: Used for React key prop and message deduplication
  - `isPanelOpen`: Matches spec FR-010b terminology
  - `citations`: Empty array for user messages, populated for assistant messages
- [x] T016 [US1] Create `fullstack/frontend-book/src/components/Chatbot/index.jsx` with floating button
- [x] T017 [US1] Create `fullstack/frontend-book/src/components/Chatbot/ChatPanel.jsx` with message list and input
- [x] T018 [US1] Create `fullstack/frontend-book/src/components/Chatbot/ChatMessage.jsx` with user/assistant bubbles
- [x] T019 [US1] Create `fullstack/frontend-book/src/components/Chatbot/Chatbot.module.css` with theme variables
- [x] T020 Install focus-trap-react: `cd fullstack/frontend-book && npm install focus-trap-react@^10.0.0`

---

## Phase 6: Docusaurus Integration (US1)

- [x] T021 [US1] Swizzle Root: `cd fullstack/frontend-book && npm run swizzle @docusaurus/theme-classic Root -- --wrap` *(created manually)*
- [x] T022 [US1] Edit `fullstack/frontend-book/src/theme/Root.js` to wrap with ChatProvider and render Chatbot
- [x] T023 [US1] Add customFields.backendUrl to `fullstack/frontend-book/docusaurus.config.js` and access via useDocusaurusContext:
  ```jsx
  import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
  const {siteConfig} = useDocusaurusContext();
  const backendUrl = siteConfig.customFields.backendUrl;
  ```
- [x] T024 [US1] Implement `useChatApi` hook in `fullstack/frontend-book/src/components/Chatbot/useChatApi.js`:
  - fetch POST to /api/chat with ChatRequest body
  - 30s timeout via AbortController
  - auto-retry once immediately on timeout (total wait: up to 60s before manual retry)
  - parse ChatResponse → {answer, citations, error}
  - handle error types: network, backend, timeout, rate_limit (429), cold_start
- [x] T024b [US1] Implement cold start detection in useChatApi: if request duration >10s, update loading message to 'Server starting, this may take up to 60 seconds...' (ref: spec FR-014b)
- [x] T025 [US1] Add loading state ("Thinking...") and render citations using `Link` from `@docusaurus/Link` in ChatMessage.jsx for client-side navigation
- [ ] T026 [US1] Validate: Widget opens, submits question, shows response with citations

---

## Phase 7: Multi-turn Conversation (US2)

- [x] T027 [US2] Pass conversation_history in ChatContext to API requests *(implemented in ChatPanel.jsx and useChatApi.js)*
- [x] T028 [US2] Add "Clear Chat" button in ChatPanel.jsx *(implemented with trash icon in header)*
- [ ] T029 [US2] Validate: Ask follow-up "Tell me more" and verify context retention

---

## Phase 8: Navigation Persistence (US3)

- [x] T030 [US3] Verify ChatContext persists across navigation (Root wrapper handles this) *(implemented via Root.js wrapper)*
- [x] T031 [US3] Add localStorage save/load in ChatContext with schema version check: if version !== '1.0', clear localStorage and notify user 'Chat history cleared due to update' *(implemented in ChatContext.jsx)*
- [ ] T032 [US3] Validate: Navigate 10+ pages, verify conversation persists

---

## Phase 9: Theme Support (US4)

- [x] T033 [US4] Update Chatbot.module.css to use --ifm-background-color, --ifm-font-color-base, --ifm-color-primary *(implemented with all theme vars)*
- [ ] T034 [US4] Validate: Toggle theme and verify widget colors update

---

## Phase 10: Error Handling (US1/US5)

- [x] T035 [US1] Add manual retry button UI for all error states (timeout logic already implemented in T024) *(implemented in ChatMessage.jsx)*
- [x] T036 [US1] Verify auto-retry behavior: after first 30s timeout, retry once immediately, then show manual retry button if second attempt fails *(implemented in useChatApi.js)*
- [x] T037 [US1] Handle 429 rate limit with specific error message: "Too many requests. Please wait 60 seconds." *(implemented in ChatPanel.jsx getErrorMessage)*
- [ ] T038 [US5] Out-of-scope queries: Verify RAG agent system prompt handles gracefully (test added to T048 checklist)
- [ ] T039 Validate: Test error states (disconnect backend, verify messages)

---

## Phase 11: Mobile & Accessibility

- [x] T040 [P] Add CSS @media (max-width: 767px) for full-screen modal in Chatbot.module.css *(implemented with full-screen panel on mobile)*
- [x] T041 [P] Add ARIA labels to button, dialog, input in Chatbot components:
  ```jsx
  <button aria-label="Open chat assistant">💬</button>
  <div role="dialog" aria-label="Chat with book assistant">
  <input aria-label="Type your question" />
  <div role="log" aria-live="polite">{messages}</div>
  ```
  *(implemented in all components)*
- [x] T042 Implement keyboard nav (Tab, Enter, Escape) and focus trap with focus-trap-react *(implemented in ChatPanel.jsx)*
- [ ] T043 Validate: Test mobile viewport and keyboard-only navigation

---

## Phase 12: Production Build

- [x] T044 Run `cd fullstack/frontend-book && npm run build` *(build succeeded)*
- [ ] T045 Verify Chatbot in build output, test with `npm run serve`

---

## Phase 13: Testing

- [ ] T046 Create `fullstack/backend/test/test_api.py` with pytest tests for /health and /api/chat (target >80% coverage)
- [ ] T047 Run tests: `cd fullstack/backend && pytest test/test_api.py --cov=app --cov-fail-under=80 --cov-report=term-missing -v`
  - `--cov-fail-under=80`: Ensures build fails if coverage drops below 80% target
- [ ] T047b Measure response times for 10 queries, validate SC-001: avg warm-start <15s, document cold-start
- [ ] T048 Create `specs/003-fastapi-chatbot/checklists/testing.md` with manual E2E checklist:
  - [ ] Chat opens on button click
  - [ ] Message submits and response displays
  - [ ] Citations render as clickable links
  - [ ] 5-message conversation flow works
  - [ ] Dark/light mode switching works
  - [ ] Conversation persists across 10+ navigations
  - [ ] Error states: backend down, rate limit, timeout
  - [ ] Out-of-scope query handled gracefully (ask 'What's the weather today?') - validates US5
  - [ ] Mobile viewport (< 768px)
  - [ ] Keyboard navigation only

---

## Phase 14: Deployment

- [ ] T049 Deploy backend to HF Space: upload app.py, api_models.py, validate_config.py, agent.py, retrieval.py, models.py, requirements.txt. Configure secrets: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, OPENROUTER_API_KEY, BOOK_BASE_URL, ALLOWED_ORIGINS
- [ ] T050 Update docusaurus.config.js with production backendUrl
- [ ] T051 Deploy frontend to GitHub Pages
- [ ] T052 Validate: Full E2E production test

---

## Dependencies

```
T001-T002 (Phase 1: Gates)
    │
    ▼
T003a-T003f (Phase 2: Research)
    │
    ▼
T004-T011 (Phase 3: Backend API, includes T010b OpenAPI)
    │
    ├─► T012-T014 (Phase 4: HF Setup)
    │
    ▼
T015-T020 (Phase 5: Frontend Widget)
    │
    ▼
T021-T026 + T024b (Phase 6: Docusaurus Integration) ◄── MVP
    │
    ├─► T027-T029 (Phase 7: US2 Multi-turn)
    │
    ├─► T030-T032 (Phase 8: US3 Navigation)
    │
    ├─► T033-T034 (Phase 9: US4 Theme)
    │
    └─► T035-T039 (Phase 10: Error/US5)
           │
           ▼
    T040-T043 (Phase 11: Mobile/A11y)
           │
           ▼
    T044-T045 (Phase 12: Production Build)
           │
           ▼
    T046-T048 (Phase 13: Testing)
           │
           ▼
    T049-T052 (Phase 14: Deployment)
```

## Parallel Execution

**Phase 2 Research (T003a-T003f)**: All research tasks can be done in parallel

**Phase 3 Backend (T004, T005)**: api_models.py and validate_config.py can be written in parallel

**Phase 5 Frontend (T015-T019)**: All component files can be written in parallel before integration

**Phase 11 Mobile/A11y (T040-T041)**: CSS and ARIA can be done in parallel

---

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 62 |
| Phase 1 (Gates) | 2 tasks |
| Phase 2 (Research) | 6 tasks (T003a-T003f) |
| Phase 3 (Backend API) | 12 tasks (T004-T011 + T010b) |
| Phase 4 (HF Setup) | 4 tasks |
| Phase 5 (Frontend Widget) | 6 tasks |
| Phase 6 (Docusaurus) | 7 tasks (T021-T026 + T024b) |
| US1 (Core Q&A) | 20 tasks |
| US2 (Multi-turn) | 3 tasks |
| US3 (Navigation) | 3 tasks |
| US4 (Theme) | 2 tasks |
| US5 (Out-of-scope) | 1 task |
| Infrastructure | 33 tasks |

**MVP Scope**: T001-T026 + T024b (31 tasks) delivers US1 - single question answering with citations
- Phase 1: 2 tasks (T001-T002)
- Phase 2: 6 tasks (T003a-T003f)
- Phase 3: 12 tasks (T004-T011, T010b)
- Phase 5: 6 tasks (T015-T020)
- Phase 6: 7 tasks (T021-T026, T024b) - note T024b is cold start detection