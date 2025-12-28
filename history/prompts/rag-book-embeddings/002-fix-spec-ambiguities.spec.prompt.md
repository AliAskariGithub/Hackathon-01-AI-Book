---
id: "002"
title: "Fix spec ambiguities and missing requirements"
stage: "spec"
date_iso: "2025-12-28"
model: "claude-sonnet-4-5-20250929"
feature: "rag-book-embeddings"
branch: "1-rag-book-embeddings"
user: "Claude Code User"
command: "Fix ambiguities and missing requirements in spec.md"
labels: ["spec-refinement", "requirements", "deduplication", "logging", "idempotency"]
links:
  spec: "specs/1-rag-book-embeddings/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/1-rag-book-embeddings/spec.md"
  - "specs/1-rag-book-embeddings/checklists/requirements.md"
tests: []
---

# PROMPT

Fix ambiguities and missing requirements in spec.md

Issues to resolve:

1. FR-007 Clarity: Remove "optional" ambiguity for chunk content storage
   - Decision needed: Store full chunk text in Qdrant or only metadata with reference?
   - If optional, specify exact conditions (e.g., chunks <200 chars stored, longer chunks reference-only)

2. Tokenization Method: Replace vague assumption with concrete implementation
   - Change from "standard whitespace-based" to specify using Cohere's tokenizer
   - Ensures chunks don't exceed model limits due to tokenization mismatch

3. Deduplication Requirements: Align edge case with functional requirements
   - Either add FR-013 for duplicate detection and handling
   - Or remove from edge cases if not implementing

4. Missing Entities: Add to Key Entities section
   - QdrantCollection: Configuration and connection details
   - ValidationReport: Test results, pass/fail status, metrics

5. Idempotency: Add FR-013 or FR-014
   - Specify behavior when re-running ingestion (skip existing, overwrite, or error)
   - Prevents duplicate vectors on multiple runs

6. Logging Specification: Enhance FR-012
   - Specify log levels (INFO for progress, ERROR for failures, DEBUG for details)
   - Define what metrics get logged (pages scraped, chunks created, embeddings generated)

Apply fixes while maintaining existing structure and numbering where possible.

# RESPONSE

Applied all 6 fixes to the specification:

**1. FR-007 Clarity** - Changed from "chunk content (optional for storage efficiency)" to "full chunk text content" with explicit rationale: "The chunk text is stored to enable result display without additional API calls."

**2. Tokenization Method** - Updated assumption from "standard whitespace-based or word-based token counting" to "Cohere's official tokenizer library (cohere/tokenizer) to ensure consistent token counts between chunking and embedding API calls."

**3. Deduplication** - Added two new requirements:
- FR-013: Idempotency - "System MUST handle re-runs idempotently by checking for existing vectors before creating new ones"
- FR-014: No duplicates - "System MUST NOT store duplicate vectors for identical chunk content"
- Updated edge case to reference FR-014

**4. Missing Entities** - Added:
- QdrantCollection: collection configuration (name, dimensions, similarity metric)
- ValidationReport: test results, pass/fail counts, average scores, failing details

**5. Idempotency** - Covered by FR-013 (see #3)

**6. Logging Specification** - Enhanced FR-012 with:
- Log levels: INFO for progress, ERROR for failures, DEBUG for detailed traces
- Metrics: pages discovered, pages scraped, chunks created, embeddings generated, storage successes/failures, validation pass/fail counts

Updated checklist with revision history documenting all changes.
