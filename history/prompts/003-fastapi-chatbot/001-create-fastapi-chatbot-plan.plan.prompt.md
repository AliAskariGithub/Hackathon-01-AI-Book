---
id: 001
title: Create FastAPI Chatbot Implementation Plan
stage: plan
date: 2025-12-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-fastapi-chatbot
branch: 003-fastapi-chatbot
user: developer
command: /sp.plan
labels: ["fastapi", "chatbot", "docusaurus", "planning", "architecture"]
links:
  spec: specs/003-fastapi-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-fastapi-chatbot/plan.md (created)
  - specs/003-fastapi-chatbot/research.md (created)
  - specs/003-fastapi-chatbot/data-model.md (created)
  - specs/003-fastapi-chatbot/quickstart.md (created)
  - specs/003-fastapi-chatbot/contracts/chat-api.yaml (created)
tests:
  - N/A (planning phase)
---

## Prompt

/sp.plan FastAPI backend with Docusaurus chatbot integration

Phase 1: Backend API Development
- Create api.py with FastAPI app initialization
- Implement POST /api/chat endpoint with ChatRequest/ChatResponse models
- Integrate Spec-2 agent (import process_query from agent.py)
- Add CORS middleware for localhost:3000, localhost:8000, GitHub Pages, HuggingFace
- Transform absolute URLs to Docusaurus relative routes (/docs/page-name)
- Add error handling with user-friendly messages
- Implement health check endpoint GET /health

Phase 2: Hugging Face Deployment
- Create requirements.txt with fastapi, uvicorn, dependencies from Spec-1/2
- Create app.py entry point for HF Spaces
- Add .env configuration for API keys (Cohere, Qdrant, OpenRouter)
- Test locally with uvicorn before deployment
- Deploy to Hugging Face Space and verify endpoints

Phase 3: Frontend Chat Widget Component
- Create src/components/Chatbot/index.jsx in Docusaurus project
- Implement floating button (bottom-right, theme-aware styling)
- Implement expandable chat panel with message list
- Add ChatMessage component (user/assistant bubbles with citations)
- Integrate Docusaurus theme variables for dark/light mode
- Add conversation state management with React Context
- Implement localStorage persistence for cross-session history

Phase 4: Widget Integration with Docusaurus
- Swizzle @theme/Root component to render Chatbot globally
- Configure customFields.backendUrl in docusaurus.config.js
- Pass backend URL to Chatbot via context or props
- Ensure widget persists across page navigation (SPA behavior)

Phase 5: API Communication
- Implement fetch POST to /api/chat with conversation history
- Add loading state UI ("thinking" indicator)
- Parse ChatResponse and render citations as Docusaurus Links
- Handle errors: network timeout (30s), backend unavailable, rate limit (429)
- Implement manual retry button and single auto-retry

Phase 6: Mobile Responsiveness & Accessibility
- Add media query for <768px screens (full-screen modal)
- Implement keyboard navigation (Tab, Enter, Escape)
- Add ARIA labels and focus trap for chat panel
- Test on mobile viewport

Phase 7: Testing & Validation
- Test backend endpoint with curl/Postman
- Test widget with 5+ question conversation
- Verify citations navigate to correct Docusaurus pages
- Test dark/light mode switching
- Validate localStorage persistence across sessions
- Test error states (backend down, rate limit, network timeout)

## Response snapshot

Created comprehensive implementation plan with 5 planning artifacts:

**research.md** (10 research topics):
- R1: FastAPI + Hugging Face Spaces deployment (Docker Space with uvicorn)
- R2: Docusaurus @theme/Root swizzling for global widget
- R3: React Context + localStorage for state persistence
- R4: Citation URL transformation (absolute → relative)
- R5: CORS configuration for multiple origins
- R6: Docusaurus theme variables (--ifm-* CSS properties)
- R7: Focus trap for keyboard accessibility
- R8: Typed error handling with frontend mapping
- R9: Spec-2 agent integration pattern
- R10: Hugging Face Space file requirements

**data-model.md**:
- Backend entities: ChatRequest, ChatResponse, ChatMessage, Citation
- Client entities: ClientChatMessage, ConversationState, ErrorState
- State flow diagram and validation rules

**contracts/chat-api.yaml**:
- OpenAPI 3.0 specification
- POST /api/chat with full request/response schemas
- GET /health for monitoring
- Error types: retrieval, generation, rate_limit, validation, internal

**quickstart.md**:
- 5-minute quick start guide
- File structure overview
- Configuration reference
- HF Spaces deployment steps
- Troubleshooting guide

**plan.md** (7 implementation phases):
1. Backend API Development
2. Hugging Face Deployment Setup
3. Frontend Chat Widget
4. Docusaurus Integration
5. API Communication
6. Mobile & Accessibility
7. Testing & Polish

## Outcome

- Impact: Complete implementation plan ready for /sp.tasks
- Tests: N/A (planning phase)
- Files: 5 artifacts created
- Next prompts: /sp.tasks to generate implementation tasks
- Reflection: User provided detailed phase breakdown which informed structure

## Evaluation notes (flywheel)

- Failure modes observed: Script created 003-fastapi-chatbot but spec was in 3-fastapi-chatbot (consolidated)
- Graders run and results (PASS/FAIL): Constitution check PASS (6/6 principles)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to task generation
