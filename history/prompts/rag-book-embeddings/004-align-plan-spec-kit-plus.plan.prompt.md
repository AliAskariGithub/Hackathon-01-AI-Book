---
id: "004"
title: "Align plan.md with spec-kit-plus methodology"
stage: "plan"
date_iso: "2025-12-28"
model: "claude-sonnet-4-5-20250929"
feature: "rag-book-embeddings"
branch: "1-rag-book-embeddings"
user: "Claude Code User"
command: "Align plan.md with spec-kit-plus methodology and fix inconsistencies"
labels: ["planning", "spec-kit-plus", "methodology", "alignment"]
links:
  spec: "specs/1-rag-book-embeddings/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/1-rag-book-embeddings/plan.md"
tests: []
---

# PROMPT

Align plan.md with spec-kit-plus methodology and fix inconsistencies

Issues to resolve:

1. Phase Terminology: Rename "Implementation Phases" section
   - Change to "Implementation Steps" or "Task Breakdown"
   - Reserve "Phase" terminology for spec-kit-plus workflow phases
   - Current Phase 1-6 should become Step 1-6 or Task Groups

2. Add Research Section (before implementation)
   - Research Cohere tokenizer API for accurate token counting
   - Verify Qdrant collection schema and metadata requirements
   - Inspect target book's sitemap.xml format and structure
   - Review Cohere rate limits and batch processing best practices
   - Output: research.md file (if spec-kit-plus uses this)

3. Add Design Deliverables Section
   - Create data-model.md: Define ContentPage, ContentChunk, VectorEmbedding entities
   - Create quickstart.md: Setup instructions and first-run guide
   - Create contracts/retrieval-api.md OR remove if truly out of scope for Spec-1
   - Clarify when these are created in the workflow

4. Fix FR-014 Reference: Change "content hashes for deduplication (FR-014)"
   - Either remove reference until spec.md is updated
   - Or note as "pending FR-014 addition to spec"

5. Specify Test Queries: Add concrete validation examples
   - List 5+ example queries in plan (e.g., "What is RAG?", "How to deploy Docusaurus?")
   - Map each query to expected book section for validation

6. Remove/Clarify contracts/retrieval-api.md
   - If retrieval API is out of scope (Spec-3), remove from structure
   - If needed for internal module contracts, clarify it's for Python function signatures only

7. Add Cohere Tokenizer Integration: Insert into implementation steps
   - Add step: "Install and configure Cohere tokenizer for chunking"
   - Ensure token counting uses Cohere's method before chunking

8. Add Testing Strategy: Specify when tests are written
   - Clarify TDD approach or test-after-implementation
   - Define test coverage targets (e.g., 80% for main.py, 100% for retrieval.py)

9. Add Rollback/Recovery Strategy
   - Specify behavior for partial ingestion failures
   - Define cleanup process for incomplete collections

10. Validate Performance Goals: Add basis or mark as estimates
    - Change "<5 minutes, <500ms" to "Target: <5 minutes (estimated based on ~100 pages)"
    - Or add "to be validated during implementation"

Apply fixes while maintaining spec-kit-plus workflow structure and formatting style.

# RESPONSE

Applied all 10 fixes to plan.md:

## Changes Made

| Issue | Resolution |
|-------|------------|
| 1. Phase Terminology | Renamed "Implementation Phases" → "Implementation Steps", Phase 1-6 → Step 1-7 |
| 2. Research Section | Added "Research Summary" section with key findings table, links to research.md |
| 3. Design Deliverables | Added section documenting data-model.md, contracts/retrieval-api.md, quickstart.md |
| 4. FR-014 Reference | Verified FR-014 exists in spec.md - reference is valid |
| 5. Test Queries | Added "Validation Test Queries" section with 6 queries mapped to book sections |
| 6. Contracts Clarity | Clarified contracts/retrieval-api.md is for Python function signatures (not HTTP) |
| 7. Cohere Tokenizer | Added to Step 1 and Step 2 for installation and chunking validation |
| 8. Testing Strategy | Added section with TAI approach, coverage targets (70%/90%), test categories |
| 9. Rollback Strategy | Added "Rollback and Recovery Strategy" section with scenarios and cleanup |
| 10. Performance Goals | Changed to "Target <5 minutes (estimated for ~100 pages); to be validated" |

## New Sections Added

- **Research Summary**: Key findings table with link to research.md
- **Design Deliverables**: Lists artifacts created during /sp.plan
- **Validation Test Queries**: 6 concrete queries with expected book sections
- **Testing Strategy**: TAI approach, coverage targets, test categories
- **Rollback and Recovery Strategy**: Failure scenarios and recovery procedures

## Structure Alignment

- Constitution Check gate references updated ("before research" not "before Phase 0")
- Implementation Steps now clearly labeled as "task groups for /sp.tasks generation"
- All artifact references include which command creates them (/sp.specify, /sp.plan, /sp.tasks)
