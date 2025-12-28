# Internal API Contract: RAG Pipeline

**Feature**: 001-rag-book-embeddings
**Date**: 2025-12-28
**Type**: Internal Python module interfaces (not HTTP endpoints)

> Note: HTTP/REST endpoints will be defined in Spec-3 (FastAPI). This document defines the internal Python function interfaces for the pipeline scripts.

---

## main.py - Ingestion Module

### `ingest_book(base_url: str, collection_name: str = "book_embeddings") -> IngestionResult`

Main entry point for book content ingestion.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| base_url | str | Yes | Base URL of deployed Docusaurus book |
| collection_name | str | No | Qdrant collection name (default: "book_embeddings") |

**Returns**: `IngestionResult`
```python
@dataclass
class IngestionResult:
    success: bool
    pages_discovered: int
    pages_scraped: int
    chunks_created: int
    embeddings_generated: int
    vectors_stored: int
    duplicates_skipped: int
    errors: List[str]
    duration_seconds: float
```

**Errors**:
| Error | Cause | Recovery |
|-------|-------|----------|
| SitemapNotFoundError | sitemap.xml not accessible | Check URL, try fallback |
| CohereAPIError | Embedding API failure | Retry with backoff |
| QdrantConnectionError | Vector DB unavailable | Check credentials |

---

### `scrape_sitemap(base_url: str) -> List[str]`

Fetch and parse sitemap.xml to extract page URLs.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| base_url | str | Yes | Base URL (sitemap at {base_url}/sitemap.xml) |

**Returns**: `List[str]` - List of page URLs

**Errors**:
- `SitemapNotFoundError`: 404 or invalid XML
- `NetworkError`: Connection failure

---

### `scrape_page(url: str) -> ContentPage`

Fetch and extract content from a single page.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| url | str | Yes | Full page URL |

**Returns**: `ContentPage` (see data-model.md)

**Errors**:
- `PageNotFoundError`: 404 response
- `ContentExtractionError`: No extractable content

---

### `chunk_content(page: ContentPage, target_tokens: int = 750, overlap_tokens: int = 100) -> List[ContentChunk]`

Split page content into overlapping chunks.

**Parameters**:
| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| page | ContentPage | Yes | - | Page to chunk |
| target_tokens | int | No | 750 | Target tokens per chunk (500-1000 range) |
| overlap_tokens | int | No | 100 | Overlap between adjacent chunks |

**Returns**: `List[ContentChunk]`

**Validation**:
- Each chunk should have 500-1000 tokens (may be smaller for end-of-page)
- Overlap should be exactly `overlap_tokens` between adjacent chunks

---

### `generate_embeddings(chunks: List[ContentChunk], batch_size: int = 96) -> List[VectorEmbedding]`

Generate embeddings for chunks using Cohere API.

**Parameters**:
| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| chunks | List[ContentChunk] | Yes | - | Chunks to embed |
| batch_size | int | No | 96 | Texts per API call (max 96) |

**Returns**: `List[VectorEmbedding]`

**Rate Limiting**: Implements exponential backoff on 429 responses

---

### `store_vectors(embeddings: List[VectorEmbedding], chunks: List[ContentChunk], collection_name: str) -> StorageResult`

Store embeddings with metadata in Qdrant.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| embeddings | List[VectorEmbedding] | Yes | Embedding vectors |
| chunks | List[ContentChunk] | Yes | Corresponding chunks (for payload) |
| collection_name | str | Yes | Target collection |

**Returns**: `StorageResult`
```python
@dataclass
class StorageResult:
    stored: int
    skipped_duplicates: int
    errors: List[str]
```

**Idempotency**: Checks for existing URL+chunk_index before insert

---

## retrieval.py - Query Module

### `search(query: str, top_k: int = 5, threshold: float = 0.0, collection_name: str = "book_embeddings") -> List[RetrievalResult]`

Perform semantic search for a query.

**Parameters**:
| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| query | str | Yes | - | Natural language query |
| top_k | int | No | 5 | Number of results to return |
| threshold | float | No | 0.0 | Minimum similarity score filter |
| collection_name | str | No | "book_embeddings" | Collection to search |

**Returns**: `List[RetrievalResult]` sorted by score descending

**Example**:
```python
results = search("What is URDF?", top_k=3, threshold=0.7)
for r in results:
    print(f"{r.score:.3f} - {r.title}: {r.text[:100]}...")
```

---

### `validate(test_queries: List[str], threshold: float = 0.7, top_k: int = 3) -> ValidationReport`

Run validation against test queries.

**Parameters**:
| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| test_queries | List[str] | Yes | - | Queries to test |
| threshold | float | No | 0.7 | Minimum passing similarity |
| top_k | int | No | 3 | Results per query |

**Returns**: `ValidationReport` (see data-model.md)

**Success Criteria**: `pass_rate >= 1.0` (all queries meet threshold)

---

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| COHERE_API_KEY | Yes | Cohere API key for embeddings |
| QDRANT_URL | Yes | Qdrant Cloud cluster URL |
| QDRANT_API_KEY | Yes | Qdrant API key |
| BOOK_BASE_URL | No | Default book URL (fallback) |
| LOG_LEVEL | No | Logging level (INFO, DEBUG, ERROR) |

---

## CLI Interface

### main.py
```bash
# Full ingestion
python main.py --url https://username.github.io/repo-name

# With custom collection
python main.py --url https://... --collection my_collection

# Dry run (no storage)
python main.py --url https://... --dry-run
```

### retrieval.py
```bash
# Interactive query
python retrieval.py --query "What is forward kinematics?"

# Run validation
python retrieval.py --validate

# Custom threshold
python retrieval.py --validate --threshold 0.8
```
