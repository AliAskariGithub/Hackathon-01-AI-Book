---
id: 008
title: Fix Tasks Implementation Details and Consistency
stage: tasks
date: 2025-12-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-fastapi-chatbot
branch: 003-fastapi-chatbot
user: developer
command: (continued session)
labels: ["tasks", "fixes", "implementation-details", "consistency"]
links:
  spec: specs/003-fastapi-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-fastapi-chatbot/tasks.md (modified)
tests:
  - N/A (task specification update)
---

## Prompt

Fix task inconsistencies and add missing implementation details in tasks.md with 22 specific issues to resolve including:
- Fix localStorage schema field name in T015 (isOpen → isPanelOpen)
- Align error types in T004 with spec (network|backend|timeout|rate_limit)
- Add async verification note to T024
- Remove/clarify T035 redundancy
- Add cold start detection task T024b
- And 17 more specific fixes

## Response snapshot

Applied all 22 fixes to tasks.md:

**Header**:
- Added phase numbering alignment note

**New Tasks Added**:
- T003f: Create data-model.md documenting all entities
- T010b: Generate OpenAPI spec (moved from T004b, after endpoints implemented)
- T024b: Implement cold start detection (>10s → update message) ref: FR-014b

**Phase 2 (Research)**:
- Added T003f for data-model.md creation
- Updated count: 6 tasks (T003a-T003f)

**Phase 3 (Backend API)**:
- T004: Fixed error_type enum to match spec: `network|backend|timeout|rate_limit`
- T006: Added placeholder note for example GitHub username
- T008: Added async verification note (check process_query signature)
- T010b: Moved OpenAPI generation after all endpoints (depends on T006-T010)
- Updated count: 12 tasks

**Phase 4 (HF Setup)**:
- T013: Fixed {user} → {username}, added placeholder note
- T013b: Clarified conditional logic with documentation fallback

**Phase 5 (Frontend Widget)**:
- T015: Fixed `isOpen` → `isPanelOpen`, added field documentation

**Phase 6 (Docusaurus)**:
- T023: Added useDocusaurusContext code example
- T024: Updated auto-retry timing (60s total), added cold_start error type
- T024b: NEW - cold start detection task
- T025: Added Link import clarification
- Updated count: 7 tasks

**Phase 8 (Navigation)**:
- T031: Added schema migration logic with user notification

**Phase 10 (Error Handling)**:
- T035: Changed to "Add manual retry button UI" (timeout in T024)
- T036: Clarified auto-retry verification behavior
- T038: Moved test to T048 checklist

**Phase 13 (Testing)**:
- T047: Added `--cov-fail-under=80` flag
- T048: Added out-of-scope query test (validates US5)

**Phase 14 (Deployment)**:
- T049: Added validate_config.py to upload files, ALLOWED_ORIGINS to secrets

**Dependencies Diagram**:
- Updated T003a-T003f, added T010b note, added T024b

**Summary**:
- Total Tasks: 62 (was 59)
- MVP Scope: 31 tasks with detailed breakdown

## Outcome

- Impact: Tasks now fully aligned with spec and plan, no ambiguity
- Tests: N/A (task specification update)
- Files: 1 modified (tasks.md)
- Next prompts: /sp.implement to begin implementation
- Reflection: Cross-referencing spec/plan/tasks ensures consistency

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format validation PASS - all 22 fixes applied
- Prompt variant (if applicable): Detailed fix list with 22 items
- Next experiment (smallest change to try): Begin implementation with Phase 1 gates
