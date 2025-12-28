---
id: 002
title: Fix Plan Add Research and Deployment Phases
stage: plan
date: 2025-12-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-fastapi-chatbot
branch: 003-fastapi-chatbot
user: developer
command: manual plan update
labels: ["fastapi", "chatbot", "planning", "fix", "phases"]
links:
  spec: specs/003-fastapi-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-fastapi-chatbot/plan.md (modified)
  - specs/003-fastapi-chatbot/data-model.md (modified)
tests:
  - N/A (planning phase)
---

## Prompt

Add research phase and fix implementation gaps in plan.md

Issues to resolve (22 total):
1. Add Phase 0: Research (before Phase 1)
2. Clarify Hugging Face Deployment in Phase 2
3. Fix Phase 3 Validation Reference (remove Storybook)
4. Add Citation Link Component to Phase 5
5. Specify Swizzling Method in Phase 4 (--wrap)
6. Add Base URL Configuration to Phase 1
7. Specify Timeout Implementation in Phase 5
8. Add Error Type Definitions to Phase 1
9. Define localStorage Schema in Phase 3
10. Specify CORS Origins List in Phase 1
11. Clarify Phase 7 Testing Strategy
12. Add Build Verification Phase
13. Add HF Space Configuration to Phase 2
14. Add Environment Variable Validation to Phase 2
15. Clarify Citation Score Usage
16. Specify Focus Trap Version in Dependencies
17. Add Documentation Generation Tasks
18. Add Pre-Implementation Gate
19. Add Deployment Phase (Phase 9)
20. Add Performance Benchmarking Task
21. Update Phase Numbering
22. Add Configuration Management Validation

## Response snapshot

Updated plan.md with all 22 fixes:

**Structural Changes**:
- Added Pre-Implementation Gate (Spec-1/2 validation)
- Added Phase 0: Research (7 tasks)
- Added Phase 7: Production Build Verification
- Added Phase 8: Testing & Validation (expanded)
- Added Phase 9: Deployment
- Total phases: 0-9 (was 1-7)

**Phase 0 Research Tasks**:
1. Docusaurus Root swizzling (--eject vs --wrap)
2. HF Spaces deployment options (Docker vs Python)
3. FastAPI CORS configuration testing
4. Citation URL transformation patterns
5. focus-trap-react compatibility check
6. Update research.md
7. Update data-model.md with localStorage schema

**Phase 1 Additions**:
- ErrorResponse model with typed errors
- BOOK_BASE_URL environment variable
- get_book_base_url() helper function
- Configurable ALLOWED_ORIGINS
- OpenAPI spec generation task

**Phase 2 Additions**:
- Conditional Dockerfile creation (per Phase 0)
- validate_config.py for startup validation
- HF Space README.md
- Required env vars list

**Phase 3-6 Fixes**:
- Removed Storybook validation reference
- Specified --wrap swizzle method
- Added Link import from @docusaurus/Link
- AbortController timeout implementation
- focus-trap-react@^10.0.0 version

**Phase 7 (NEW)**: Production Build Verification
**Phase 8**: Testing with manual checklist, >80% coverage target
**Phase 9**: Deployment to HF Space and GitHub Pages

**data-model.md Updates**:
- Added localStorage schema (version 1.0)
- Added score field to Citation (pass-through only)
- Documented schema migration strategy

## Outcome

- Impact: Comprehensive plan with research phase, gates, and deployment
- Tests: N/A (planning phase)
- Files: 2 modified (plan.md, data-model.md)
- Next prompts: /sp.tasks to generate implementation tasks
- Reflection: All 22 issues addressed systematically

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All issues addressed PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks from updated plan
