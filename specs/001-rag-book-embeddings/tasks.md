# Implementation Tasks: RAG Pipeline for Book Content Embeddings

**Feature**: 001-rag-book-embeddings
**Generated**: 2025-12-28
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Overview

| Metric | Value |
|--------|-------|
| Total Tasks | 28 |
| Research/Docs | 6 |
| Setup | 5 |
| Foundational | 1 |
| US1 (Ingestion) | 9 |
| US2 (Retrieval) | 3 |
| US3 (Validation) | 2 |
| Testing | 3 |
| Documentation | 1 |
| Parallel [P] | 8 tasks |

---

## Phase 0: Research & Documentation

*Complete before implementation to validate assumptions*

- [x] T001 Research Cohere tokenizer API and document token counting method
- [x] T002 [P] Inspect target book sitemap.xml format and verify structure
- [x] T003 [P] Review Cohere rate limits (96/batch, 1000/min) and Qdrant free tier constraints (1GB)
- [x] T004 Create specs/1-rag-book-embeddings/research.md with findings from T001-T003
- [x] T005 [P] Create specs/1-rag-book-embeddings/data-model.md documenting all entities
- [x] T006 [P] Create specs/1-rag-book-embeddings/quickstart.md with setup and first-run guide

---

## Phase 1: Setup

- [x] T007 Initialize uv project in fullstack/backend/ with pyproject.toml
- [x] T008 [P] Create fullstack/backend/.env.example with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, BOOK_BASE_URL, BATCH_SIZE
- [x] T009 [P] Create fullstack/backend/.gitignore (exclude .env, __pycache__, .venv)
- [x] T010 Install dependencies: cohere, qdrant-client, beautifulsoup4, requests, python-dotenv, pytest
- [ ] T011 Verify connectivity to Cohere API and Qdrant Cloud with test credentials (user creates .env first)

---

## Phase 2: Foundational

- [x] T012 Define dataclasses in fullstack/backend/models.py:
  - ContentPage, ContentChunk, VectorEmbedding
  - QdrantCollection (config), RetrievalResult, ValidationReport, QueryResult

---

## Phase 3: User Story 1 - Content Ingestion and Embedding (P1)

**Goal**: Scrape book, chunk content, generate embeddings, store in Qdrant
**Independent Test**: Run `python main.py --url <book_url>` and verify vectors in Qdrant

- [x] T013 [US1] Implement scrape_sitemap(base_url) to fetch and parse sitemap.xml in fullstack/backend/main.py
- [x] T014 [US1] Implement scrape_page(url) to extract clean text via BeautifulSoup in fullstack/backend/main.py
- [x] T015 [US1] Configure Cohere tokenizer for token counting (required before chunking) in fullstack/backend/main.py
- [x] T016 [US1] Implement chunk_content(page) with tokenizer validation (500-1000 tokens, 100 overlap) in fullstack/backend/main.py
- [x] T017 [US1] Implement generate_embeddings(chunks) with configurable BATCH_SIZE (default 96, per Cohere API limit) and exponential backoff in fullstack/backend/main.py
- [x] T018 [US1] Implement init_collection() to create Qdrant collection (1024 dims, cosine) in fullstack/backend/main.py
- [x] T019 [US1] Implement store_vectors(embeddings, chunks) with deduplication check per FR-013 (idempotency) and FR-014 (no duplicates) in fullstack/backend/main.py
- [x] T020 [US1] Implement ingest_book(base_url) orchestrating full pipeline with logging (INFO/ERROR/DEBUG levels) in fullstack/backend/main.py
- [x] T021 [US1] Add CLI interface with --url, --collection, --dry-run flags in fullstack/backend/main.py

---

## Phase 4: User Story 2 - Semantic Retrieval (P1)

**Goal**: Query embeddings and return ranked results with metadata
**Independent Test**: Run `python retrieval.py --query "What is URDF?"` and verify results

- [x] T022 [US2] Implement search(query, top_k, threshold) returning RetrievalResult list in fullstack/backend/retrieval.py
- [x] T023 [US2] Format results with score, title, URL, text excerpt in fullstack/backend/retrieval.py
- [x] T024 [US2] Add CLI interface with --query, --top-k, --threshold flags in fullstack/backend/retrieval.py

---

## Phase 5: User Story 3 - Pipeline Validation (P2)

**Goal**: Validate retrieval quality with test queries achieving >0.7 similarity
**Independent Test**: Run `python retrieval.py --validate` and verify 100% pass rate

- [x] T025 [US3] Implement validate(test_queries, threshold) returning ValidationReport in fullstack/backend/retrieval.py
- [x] T026 [US3] Add --validate CLI flag with 6 test queries in fullstack/backend/retrieval.py:
  1. "What is URDF and how is it used?"
  2. "How do forward kinematics work?"
  3. "What sensors are used in robotics?"
  4. "How to set up Gazebo simulation?"
  5. "What is Isaac Sim architecture?"
  6. "How do AI agents plan actions?"

---

## Phase 6: Testing & Documentation

**Coverage Target**: 70% main.py, 90% retrieval.py

- [x] T027 [P] Write unit tests for main.py in fullstack/backend/test/test_main.py (scraping, chunking, deduplication)
- [x] T028 [P] Write unit tests for retrieval.py in fullstack/backend/test/test_retrieval.py (search, validation)
- [x] T029 [P] Create fullstack/backend/README.md with setup instructions, environment variables, usage examples

---

## Dependencies

```
Phase 0 (Research):
T001 → T004
T002 ─┬→ T004
T003 ─┘
T005, T006 (parallel)

Phase 1 (Setup):
T004 → T007 → T010 → T011
T008, T009 (parallel after T007)

Phase 2-3 (Foundational + US1):
T011 → T012 → T013 → T014 → T015 → T016 → T017 → T018 → T019 → T020 → T021

Phase 4 (US2):
T021 → T022 → T023 → T024

Phase 5 (US3):
T024 → T025 → T026
T012 → T025 (ValidationReport model needed)

Phase 6 (Testing/Docs):
T021 → T027
T024 → T028
T027, T028, T029 (parallel)
```

**Story Dependencies**:
- US2 requires US1 complete (needs vectors in Qdrant)
- US3 requires US2 complete (needs search() function)
- US3 requires T012 complete (ValidationReport model)

## Parallel Execution

| Phase | Parallel Tasks |
|-------|----------------|
| Phase 0 | T002+T003, T005+T006 |
| Phase 1 | T008+T009 after T007 |
| Phase 6 | T027+T028+T029 |

## MVP Scope

**Minimum Viable Product**: Complete Phase 0-3 (Research + US1)
- Research validated
- Working ingestion pipeline
- Vectors stored in Qdrant
- Verifiable via Qdrant dashboard

**Full Feature**: Add Phase 4-6
- Search functionality (US2)
- Validation with test queries (US3)
- Tests and documentation