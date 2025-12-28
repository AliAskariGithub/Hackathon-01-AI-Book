---
id: "006"
title: "Fix tasks.md add missing tasks"
stage: "tasks"
date_iso: "2025-12-28"
model: "claude-sonnet-4-5-20250929"
feature: "rag-book-embeddings"
branch: "1-rag-book-embeddings"
user: "Claude Code User"
command: "Add missing tasks and fix inconsistencies in tasks.md"
labels: ["tasks", "fixes", "research", "testing", "documentation"]
links:
  spec: "specs/1-rag-book-embeddings/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/1-rag-book-embeddings/tasks.md"
tests: []
---

# PROMPT

Add missing tasks and fix inconsistencies in tasks.md

Issues to resolve:
1. Add Research/Documentation Tasks (before T001)
2. Fix T005 Entity Definition: Add all missing entities
3. Add Tokenizer Setup Task (before T008)
4. Add Testing Tasks for each phase
5. Add Documentation Task
6. Add Environment Verification Task (after T002)
7. Fix T011 Deduplication Reference
8. Specify Test Queries in T018
9. Justify/Configure Batch Size in T009
10. Update Dependencies Graph
11. Update Total Task Count
12. Clarify T005 Blocking Dependencies

# RESPONSE

Updated tasks.md from 18 to 28 tasks:

| Change | Before | After |
|--------|--------|-------|
| Total Tasks | 18 | 28 |
| Phases | 5 | 7 (added Phase 0: Research, Phase 6: Testing) |
| Parallel [P] | 6 | 8 |

**Added Tasks**:
- T001-T006: Research & Documentation phase
- T011: Environment verification
- T015: Cohere tokenizer configuration
- T026: 6 concrete test queries listed
- T027-T028: Unit tests for main.py and retrieval.py
- T029: README.md documentation

**Fixes Applied**:
- T012: All entities listed (ContentPage, ContentChunk, VectorEmbedding, QdrantCollection, RetrievalResult, ValidationReport, QueryResult)
- T017: BATCH_SIZE now configurable with default 96
- T019: FR-013/FR-014 references kept (exist in spec.md)
- Dependencies graph updated with all new tasks
- T012 → T025 dependency added for ValidationReport model
