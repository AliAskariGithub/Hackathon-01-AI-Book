# Implementation Plan: RAG Pipeline for Book Content Embeddings and Retrieval

**Branch**: `001-rag-book-embeddings` | **Date**: 2025-12-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/1-rag-book-embeddings/spec.md`

## Summary

Build a RAG pipeline that scrapes the deployed Docusaurus book from GitHub Pages, generates vector embeddings using Cohere's embed-english-v3.0 model, stores them in Qdrant Cloud with metadata (title, URL, chunk content), and provides semantic retrieval with >0.7 similarity validation. The implementation uses two Python scripts: `main.py` for ingestion and `retrieval.py` for query testing.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere (v5+), qdrant-client (v1.7+), beautifulsoup4 (v4.12+), requests, python-dotenv
**Storage**: Qdrant Cloud (free tier) for vectors; no relational database needed
**Testing**: pytest for unit/integration tests
**Target Platform**: Local development (scripts), deployable to any Python environment
**Project Type**: Backend scripts (single project structure)
**Performance Goals**: Target <5 minutes for full ingestion (estimated for ~100 pages, ~2000 chunks); retrieval <500ms per query (to be validated during implementation)
**Constraints**: Free tier limits (Qdrant: 1GB storage, Cohere: rate limits); environment variables for credentials
**Scale/Scope**: ~50-100 pages, ~500-2000 chunks, single collection

## Constitution Check

*GATE: Must pass before research. Re-check after design deliverables.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Spec-First Development | PASS | Feature spec completed before implementation plan |
| II. AI-Assisted Development | PASS | Plan generated with Claude Code assistance |
| III. Technical Accuracy | PASS | Using official Cohere/Qdrant APIs; verified in research.md |
| IV. Reproducible Workflows | PASS | Environment setup via uv; .env.example provided |
| V. Clean Architecture | PASS | No hardcoded secrets; env vars for all credentials |
| VI. Modular Code | PASS | Separate ingestion (main.py) and retrieval (retrieval.py) modules |

**Technology Stack Alignment**:
- Qdrant Cloud: Constitution-specified vector storage
- Python/FastAPI ecosystem: Constitution-compatible (FastAPI for future Spec-3)
- GitHub Pages: Target book deployment matches constitution

## Project Structure

### Documentation (this feature)

```text
specs/1-rag-book-embeddings/
├── spec.md              # Feature specification (/sp.specify)
├── plan.md              # This file (/sp.plan)
├── research.md          # Research output (/sp.plan)
├── data-model.md        # Design output (/sp.plan)
├── quickstart.md        # Setup guide (/sp.plan)
├── contracts/           # Internal API contracts (/sp.plan)
│   └── retrieval-api.md # Python function signatures (not HTTP endpoints)
├── checklists/
│   └── requirements.md  # Quality checklist
└── tasks.md             # Implementation tasks (/sp.tasks)
```

### Source Code (repository root)

```text
fullstack/
├── frontend-book/       # Existing Docusaurus book (target for scraping)
│   ├── docs/            # Book content pages
│   └── ...
└── backend/             # NEW: RAG pipeline backend
    ├── main.py          # Content ingestion + embedding
    ├── retrieval.py     # Query testing + validation
    ├── test/
    │   ├── test_main.py
    │   └── test_retrieval.py
    ├── .env.example     # Environment variable template
    ├── .env             # Local credentials (gitignored)
    ├── pyproject.toml   # uv project configuration
    └── README.md        # Backend setup instructions
```

**Structure Decision**: Backend scripts in `fullstack/backend/` following the web application pattern from constitution (frontend + backend separation). The backend is a standalone Python project managed with `uv`.

## Complexity Tracking

> No violations requiring justification. Design follows minimal complexity principle.

---

## Research Summary

*Full details in [research.md](./research.md)*

### Key Findings

| Topic | Decision | Rationale |
|-------|----------|-----------|
| Embedding Model | embed-english-v3.0 | 1024 dimensions, optimized for semantic search |
| Token Counting | Cohere tokenizer API | Ensures consistency with embedding API limits |
| Chunking Strategy | Recursive split + validation | Respects semantic boundaries, validates token count |
| HTML Extraction | BeautifulSoup + Docusaurus selectors | Targets `<article>` content, removes nav/footer |
| Rate Limiting | Exponential backoff + batch processing | 96 texts per batch, handles 429 responses |
| Deduplication | SHA-256 content hash + URL/index lookup | Implements FR-013, FR-014 |

### Sitemap Structure (Verified)

Docusaurus generates standard sitemap.xml at `/sitemap.xml`:
```xml
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
  <url><loc>https://site.github.io/docs/intro</loc></url>
  <url><loc>https://site.github.io/docs/module-1/topic</loc></url>
  ...
</urlset>
```

### Cohere API Constraints

- **Batch limit**: 96 texts per embed call
- **Token limit**: 512 tokens per text (chunks >512 must be split)
- **Rate limit**: 1000 calls/minute (free tier)
- **Input types**: `search_document` for indexing, `search_query` for retrieval

---

## Design Deliverables

*Created during /sp.plan execution*

### 1. data-model.md

Defines entity structures for the pipeline:
- **ContentPage**: Scraped page with URL, title, raw content
- **ContentChunk**: Chunked segment with token count, index, parent reference
- **VectorEmbedding**: 1024-dimensional vector with model metadata
- **QdrantPayload**: Stored metadata (url, title, text, chunk_index, content_hash)
- **RetrievalResult**: Query result with score and source metadata
- **ValidationReport**: Test results with pass/fail counts

### 2. contracts/retrieval-api.md

Internal Python function signatures (NOT HTTP endpoints - those are Spec-3):
- `ingest_book(base_url, collection_name)` → IngestionResult
- `scrape_sitemap(base_url)` → List[str]
- `chunk_content(page, target_tokens, overlap_tokens)` → List[ContentChunk]
- `generate_embeddings(chunks, batch_size)` → List[VectorEmbedding]
- `search(query, top_k, threshold)` → List[RetrievalResult]
- `validate(test_queries, threshold)` → ValidationReport

### 3. quickstart.md

Setup and first-run guide:
1. Initialize uv project in `fullstack/backend/`
2. Configure .env with API credentials
3. Verify Cohere and Qdrant connectivity
4. Run ingestion against target book
5. Execute validation queries

---

## Validation Test Queries

*Concrete queries for SC-004 validation (>0.7 similarity threshold)*

| Query | Expected Book Section | Purpose |
|-------|----------------------|---------|
| "What is URDF and how is it used?" | Module 2: URDF Mapping | Core concept retrieval |
| "How do forward kinematics work?" | Module 2: Forward/Inverse Kinematics | Technical explanation |
| "What sensors are used in robotics?" | Module 4: Perception | Breadth coverage |
| "How to set up Gazebo simulation?" | Module 3: Gazebo Setup | Tutorial content |
| "What is Isaac Sim architecture?" | Module 5: Isaac Sim Overview | NVIDIA-specific content |
| "How do AI agents plan actions?" | Module 6: Cognitive Planning | Advanced topic |

**Validation Criteria**:
- Each query must return at least 1 result with score > 0.7
- Top result should be from expected book section (manual verification)
- Pass rate must be 100% (6/6 queries)

---

## Testing Strategy

### Approach: Test-After-Implementation (TAI)

Given the exploratory nature of RAG pipelines and external API dependencies, tests are written after core functionality is implemented.

### Coverage Targets

| Module | Target | Rationale |
|--------|--------|-----------|
| main.py | 70% | Focus on chunking, deduplication logic; mock API calls |
| retrieval.py | 90% | Critical path; test search and validation thoroughly |
| Integration | 5 scenarios | End-to-end with real APIs (requires credentials) |

### Test Categories

1. **Unit Tests** (mocked APIs):
   - Chunking produces correct token ranges
   - Content hash generation is deterministic
   - Deduplication logic skips existing vectors

2. **Integration Tests** (real APIs):
   - Sitemap parsing returns valid URLs
   - Embedding generation succeeds for sample text
   - Qdrant storage and retrieval roundtrip

3. **Validation Tests**:
   - All 6 test queries meet threshold
   - ValidationReport accurately reflects results

---

## Rollback and Recovery Strategy

### Partial Ingestion Failure

**Scenario**: Network failure or API error during ingestion after some vectors stored.

**Recovery**:
1. Idempotent re-runs (FR-013): Check existing vectors before insert
2. Content hash comparison: Skip already-processed chunks
3. Resume from last successful page (log page URLs as processed)

**Cleanup** (if needed):
```python
# Delete all vectors in collection for fresh start
client.delete_collection(collection_name="book_embeddings")
client.create_collection(...)
```

### Validation Failure

**Scenario**: Similarity scores below threshold.

**Recovery**:
1. Review chunk boundaries (may need adjustment)
2. Verify HTML extraction captured content correctly
3. Check for empty or minimal chunks
4. Re-run ingestion with adjusted parameters

### Rollback Checklist

- [ ] Collection can be deleted and recreated
- [ ] No persistent state outside Qdrant
- [ ] Re-ingestion is safe (idempotent)
- [ ] Logs capture enough context for debugging

---

## Implementation Steps

*Task groups for /sp.tasks generation*

### Step 1: Environment Setup
- Initialize `fullstack/backend/` with uv
- Install dependencies: cohere, qdrant-client, beautifulsoup4, requests, python-dotenv
- Install and configure Cohere tokenizer for accurate token counting
- Create .env.example with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY
- Verify connectivity to deployed book URL and APIs

### Step 2: Content Extraction (main.py)
- Fetch sitemap.xml from deployed book URL
- Parse sitemap to extract all page URLs
- Crawl each page and extract clean text (remove nav, footer, boilerplate)
- Implement chunking with Cohere tokenizer validation (500-1000 tokens, 100-token overlap)
- Store page metadata (title, URL) with each chunk
- Generate SHA-256 content hash for each chunk (FR-014 deduplication)

### Step 3: Embedding Generation (main.py)
- Initialize Cohere client from environment
- Batch chunks for embedding API (max 96 per call)
- Implement exponential backoff for rate limit handling
- Use `input_type="search_document"` for indexing

### Step 4: Vector Storage (main.py)
- Initialize Qdrant collection with cosine similarity (1024 dimensions)
- Check for existing vectors before insertion (FR-013 idempotency)
- Store vectors with metadata payload (url, title, text, chunk_index, content_hash)
- Log metrics: pages scraped, chunks created, embeddings generated, storage successes/failures

### Step 5: Retrieval Implementation (retrieval.py)
- Query function: text → Cohere embedding (`input_type="search_query"`) → Qdrant search
- Return top-k results with similarity scores
- Include source metadata in results (title, URL, chunk text)

### Step 6: Validation (retrieval.py)
- Implement validation with 6 test queries (see Validation Test Queries)
- Run validation against similarity threshold (0.7)
- Generate ValidationReport with pass/fail counts and average similarity
- Log detailed results for debugging

### Step 7: Testing
- Write unit tests for chunking and deduplication logic
- Write integration tests for API interactions
- Achieve coverage targets (70% main.py, 90% retrieval.py)

---

## Risk Assessment

| Risk | Mitigation |
|------|------------|
| Cohere rate limits during large ingestion | Batch processing (96 texts), exponential backoff |
| Sitemap unavailable or malformed | Fallback to homepage link crawling; clear error messages |
| Qdrant free tier storage limits | Monitor collection size; efficient chunk sizing |
| Token count mismatch (chunking vs API) | Use Cohere tokenizer for all counting |
| Partial ingestion failure | Idempotent re-runs with deduplication |

## Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| cohere | >=5.0 | Embedding generation API |
| qdrant-client | >=1.7 | Vector storage client |
| beautifulsoup4 | >=4.12 | HTML parsing |
| requests | >=2.31 | HTTP client |
| python-dotenv | >=1.0 | Environment variable loading |
| pytest | >=7.0 | Testing framework |

## Success Metrics

- [ ] 100% of sitemap pages scraped successfully (SC-001)
- [ ] All chunks within 500-1000 token range (SC-002)
- [ ] Zero duplicate vectors in collection (FR-014)
- [ ] >0.7 similarity score for all 6 validation queries (SC-004)
- [ ] No credentials exposed in code or logs (SC-006)
- [ ] Test coverage: 70% main.py, 90% retrieval.py
