---
id: 007
title: Fix Plan Implementation Details and Clarifications
stage: plan
date: 2025-12-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-fastapi-chatbot
branch: 003-fastapi-chatbot
user: developer
command: (continued session)
labels: ["plan", "fixes", "implementation-details", "clarifications"]
links:
  spec: specs/003-fastapi-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-fastapi-chatbot/plan.md (modified)
tests:
  - N/A (planning document update)
---

## Prompt

Fix minor clarifications and complete implementation details in plan.md with 18 specific issues to resolve including:
- Clarify async handling in Phase 1 Task 6
- Add schema migration strategy to Phase 3 Task 1
- Clarify auto-retry timing in Phase 5 Task 6
- Add coverage enforcement flag to Phase 8 Task 1
- Add cold start detection task to Phase 5
- And 13 more specific fixes

## Response snapshot

Applied all 18 fixes to plan.md:

**Pre-Implementation Gate**:
- Added expected output examples for Spec-1 and Spec-2 validation commands

**Phase 0 (Research)**:
- Updated localStorage schema with `isPanelOpen` and `citations` fields (ref: FR-010b)

**Phase 1 (Backend API)**:
- Task 6: Added async handling guidance with code check instructions (ref: FR-002)
- Task 10: Added task dependencies note (Tasks 6-9 must complete first)
- Task 3: Added placeholder note for example URLs

**Phase 2 (HF Setup)**:
- Task 5-6: Fixed {user} to {username} for consistency, added placeholder notes

**Phase 3 (Frontend Widget)**:
- Task 1: Added schema migration strategy with version check and user notification

**Phase 4 (Docusaurus Integration)**:
- Task 4: Added useDocusaurusContext code example

**Phase 5 (API Communication)**:
- Task 6: Added auto-retry timing details (30s + 30s = 60s total)
- Task 7 (NEW): Added cold start detection task (>10s → update message)
- Task 9: Added cold_start to error types

**Phase 6 (Accessibility)**:
- Task 2: Added keyboard accessibility note for all buttons

**Phase 8 (Testing)**:
- Task 1: Added `--cov-fail-under=80` flag to pytest command
- Task 2: Added out-of-scope query test to checklist

**Phase 9 (Deployment)**:
- Task 1: Added validate_config.py to upload files list
- Tasks 1-6: Fixed all {user} placeholders to {username}, added replacement notes

**Dependencies Section**:
- Added Python 3.11+ to External dependencies

**Risk Mitigation Table**:
- Added research findings risk with mitigation strategy

**Constitution Check**:
- Fixed api.py → app.py reference
- Updated FR count from 25 to 26

## Outcome

- Impact: Plan now has complete implementation details for all phases
- Tests: N/A (planning document update)
- Files: 1 modified (plan.md)
- Next prompts: /sp.implement to begin implementation
- Reflection: Detailed plans reduce implementation ambiguity and rework

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format validation PASS - all 18 fixes applied
- Prompt variant (if applicable): Detailed fix list with 18 items
- Next experiment (smallest change to try): Begin implementation with Phase 1 gates
