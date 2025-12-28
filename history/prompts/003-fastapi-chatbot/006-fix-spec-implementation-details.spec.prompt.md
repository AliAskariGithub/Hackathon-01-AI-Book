---
id: 006
title: Fix Spec Implementation Details and Consistency
stage: spec
date: 2025-12-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-fastapi-chatbot
branch: 003-fastapi-chatbot
user: developer
command: (continued session)
labels: ["spec", "fixes", "implementation-details", "consistency"]
links:
  spec: specs/003-fastapi-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-fastapi-chatbot/spec.md (modified)
tests:
  - N/A (specification update)
---

## Prompt

Fix filename inconsistency and clarify implementation details in spec.md with 20 specific issues to resolve including:
- Resolve app.py vs api.py filename inconsistency
- Add URL transformation configuration to FR-009c
- Clarify citation score field usage
- Specify Root swizzling method in FR-006c
- Define localStorage schema in FR-010b
- Add new FRs: FR-003c, FR-014b, FR-026
- Update Dependencies, Assumptions, Entities sections
- And 13 more specific fixes

## Response snapshot

Applied all 20 fixes to spec.md:

**Filename Consistency**: Verified `app.py` used consistently (no api.py references found)

**New Functional Requirements Added**:
- FR-003c: ALLOWED_ORIGINS env var configuration with default
- FR-014b: Cold start detection (>10s) with specific message
- FR-026: Startup environment variable validation

**Updated Functional Requirements**:
- FR-002: Added async handling guidance and signature verification note
- FR-006c: Added --wrap method command and rationale
- FR-009c: Added BOOK_BASE_URL env var and transformation example
- FR-010b: Added full localStorage schema with version field
- FR-016: Added build-time loading clarification and .env example
- FR-018b: Added timing details (30s + 30s = 60s total)

**Updated Entities**:
- ChatMessage: Added citations field for assistant messages
- Citation: Clarified score is NOT displayed in MVP
- ConversationState: NEW entity with full localStorage schema
- ErrorState: Added 'cold_start' type

**Updated Sections**:
- SC-003: Added measurement method (manual E2E with 20+ citations)
- Assumptions: Added HF Space naming pattern, env vars section
- Dependencies: Added validation commands, Docusaurus version clarity, full env vars list
- Out of Scope: Removed redundant item, added citation scores and automated E2E

**FR Count**: 26 functional requirements (was 25)

## Outcome

- Impact: Spec now has complete implementation details for all requirements
- Tests: N/A (specification update)
- Files: 1 modified (spec.md)
- Next prompts: /sp.implement to begin implementation
- Reflection: Detailed specs reduce implementation ambiguity

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format validation PASS - all 20 fixes applied
- Prompt variant (if applicable): Detailed fix list with 20 items
- Next experiment (smallest change to try): Begin implementation with Phase 1 gates
