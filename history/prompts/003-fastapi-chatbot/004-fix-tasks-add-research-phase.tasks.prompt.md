---
id: 004
title: Fix Tasks Add Research Phase and Specifications
stage: tasks
date: 2025-12-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-fastapi-chatbot
branch: 003-fastapi-chatbot
user: developer
command: manual task update
labels: ["fastapi", "chatbot", "tasks", "fix", "research"]
links:
  spec: specs/003-fastapi-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-fastapi-chatbot/tasks.md (modified)
tests:
  - N/A (task generation phase)
---

## Prompt

Add research phase and fix task specification gaps in tasks.md

Issues to resolve (23 total):
1. Add Phase 0: Research (before Phase 1)
2. Clarify T003 Error Types
3. Specify T006 Base URL Source
4. Complete T011 Dependencies List
5. Add ALLOWED_ORIGINS Example to T012
6. Specify T014 localStorage Schema
7. Detail T023 useChatApi Hook Functionality
8. Fix T037 Out-of-Scope Handling
9. Add Documentation Generation Tasks
10. Add Performance Benchmarking Task
11. Add Dockerfile Decision Task
12. Update Phase Numbering
13. Renumber Tasks Sequentially
14. Add OpenAPI Generation Task
15. Specify Testing Coverage Target
16. Add ARIA Labels Details
17. Detail Manual Testing Checklist
18. Add Quickstart Documentation Task
19. Update Dependencies Diagram
20. Update Parallel Execution Section
21. Update Summary Totals
22. Add Conditional Dockerfile Task
23. Update MVP Scope

## Response snapshot

Updated tasks.md with all 23 fixes:

**Structural Changes**:
- Added Phase 2: Research with 5 tasks (T003a-T003e)
- Renumbered phases: Phase 1 (Gates) → Phase 2 (Research) → Phase 3-14
- Total phases: 14 (was 13)
- Total tasks: 59 (was 51)

**Phase 2 Research Tasks**:
- T003a: Docusaurus Root swizzling research
- T003b: HF Spaces deployment research
- T003c: FastAPI CORS testing
- T003d: Citation URL transformation research
- T003e: focus-trap-react compatibility check

**Phase 3 Backend Enhancements**:
- T004: Clarified ErrorResponse with error_type enum: `retrieval|generation|rate_limit|validation|internal`
- T004b: Added OpenAPI spec generation task
- T007: Specified base_url from BOOK_BASE_URL env var
- T009b: Added quickstart.md documentation task

**Phase 4 HF Setup Enhancements**:
- T012: Complete requirements.txt with version specs
- T013: ALLOWED_ORIGINS example in .env.example
- T013b: Conditional Dockerfile task (if T003b requires Docker Space)

**Phase 5 Frontend Enhancements**:
- T015: Full localStorage schema with version "1.0"

**Phase 6 Docusaurus Enhancements**:
- T024: Detailed useChatApi hook functionality (fetch, timeout, retry, error handling)

**Phase 10 Error/US5 Fix**:
- T038: Changed from "no code change needed" to "verify RAG agent handles + add test case"

**Phase 11 Accessibility Enhancements**:
- T041: Added ARIA labels code example

**Phase 13 Testing Enhancements**:
- T046: Added >80% coverage target
- T047b: Added performance benchmarking task
- T048: Detailed manual E2E checklist

**Updated Summary**:
- Total Tasks: 59 (was 51)
- MVP Scope: T001-T026 (34 tasks)

## Outcome

- Impact: Complete task breakdown with research phase and detailed specifications
- Tests: N/A (task generation phase)
- Files: 1 modified (tasks.md)
- Next prompts: /sp.implement
- Reflection: All 23 issues addressed, tasks now have sufficient detail for implementation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format validation PASS (all tasks have checkbox, ID, file paths)
- Prompt variant (if applicable): Fix variant addressing 23 specific issues
- Next experiment (smallest change to try): Start implementation with /sp.implement
