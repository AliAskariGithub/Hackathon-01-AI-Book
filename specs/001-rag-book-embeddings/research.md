# Research: RAG Pipeline for Book Content Embeddings

**Feature**: 001-rag-book-embeddings
**Date**: 2025-12-28
**Status**: Complete

## Research Tasks

### 1. Cohere Embedding Model Selection

**Decision**: Use `embed-english-v3.0` with `input_type="search_document"` for indexing and `input_type="search_query"` for retrieval.

**Rationale**:
- embed-english-v3.0 produces 1024-dimensional vectors optimized for semantic search
- The `input_type` parameter improves retrieval accuracy by differentiating document vs query embeddings
- Free tier supports up to 1000 API calls/minute (sufficient for book ingestion)
- Model supports up to 512 tokens per input (our chunks target 500-1000, will split if needed)

**Alternatives Considered**:
- `embed-multilingual-v3.0`: Same dimensions but unnecessary for English-only content
- `embed-english-light-v3.0`: Smaller vectors (384-dim) but lower quality for semantic search
- OpenAI embeddings: Higher cost, external dependency

**Implementation Notes**:
```python
# Indexing
embeddings = cohere.embed(texts=chunks, model="embed-english-v3.0", input_type="search_document")

# Retrieval
query_embedding = cohere.embed(texts=[query], model="embed-english-v3.0", input_type="search_query")
```

---

### 2. Qdrant Collection Configuration

**Decision**: Create collection with cosine similarity, 1024 dimensions, and optimized payload indexing.

**Rationale**:
- Cosine similarity is standard for text embeddings (normalizes vector magnitudes)
- 1024 dimensions match Cohere embed-english-v3.0 output
- Payload indexing on `url` enables deduplication checks (FR-013)
- Free tier: 1GB storage, ~100k vectors (sufficient for book content)

**Configuration**:
```python
client.create_collection(
    collection_name="book_embeddings",
    vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
)
# Create payload index for URL-based deduplication
client.create_payload_index(
    collection_name="book_embeddings",
    field_name="url",
    field_schema=PayloadSchemaType.KEYWORD,
)
```

**Alternatives Considered**:
- Dot product similarity: Requires normalized vectors; cosine handles automatically
- Euclidean distance: Less suitable for high-dimensional text embeddings
- Pinecone: Higher cost, similar functionality

---

### 3. Chunking Strategy

**Decision**: Use recursive character text splitter with Cohere tokenizer validation.

**Rationale**:
- Target 500-1000 tokens with 100-token overlap per spec (FR-004)
- Cohere embed-english-v3.0 has 512 token limit per API call
- Chunks exceeding 512 tokens must be split further before embedding
- Content hash (SHA-256 of chunk text) enables deduplication

**Implementation Approach**:
1. Extract clean text from HTML (BeautifulSoup)
2. Split by paragraphs/sentences to preserve meaning
3. Validate token count using Cohere tokenizer
4. Merge small chunks or split large ones to meet target range
5. Generate content hash for each chunk

**Token Counting**:
```python
import cohere
co = cohere.Client(api_key)
tokens = co.tokenize(text=chunk, model="embed-english-v3.0")
token_count = len(tokens.tokens)
```

**Alternatives Considered**:
- Fixed character splitting: Doesn't respect semantic boundaries
- LangChain RecursiveCharacterTextSplitter: Good but adds dependency
- Custom sentence-based: More control but more complex

---

### 4. HTML Content Extraction

**Decision**: Use BeautifulSoup with targeted CSS selectors for Docusaurus structure.

**Rationale**:
- Docusaurus uses consistent HTML structure with identifiable content areas
- Main content typically in `<article>` or `.markdown` class
- Need to exclude: navigation, footer, table of contents, code blocks (optional)

**Selectors for Docusaurus**:
```python
# Extract main content
content = soup.select_one('article') or soup.select_one('.markdown')

# Remove unwanted elements
for selector in ['nav', 'footer', '.table-of-contents', '.pagination-nav']:
    for element in content.select(selector):
        element.decompose()

# Get clean text
text = content.get_text(separator='\n', strip=True)
```

**Alternatives Considered**:
- Readability/newspaper3k: Overkill for known structure
- Regex: Fragile, hard to maintain
- Trafilatura: Good but additional dependency

---

### 5. Sitemap Parsing

**Decision**: Parse sitemap.xml directly using requests + xml.etree.ElementTree.

**Rationale**:
- Docusaurus generates standard sitemap.xml at `/sitemap.xml`
- Standard XML format with `<url><loc>` elements
- No additional dependencies needed

**Implementation**:
```python
import xml.etree.ElementTree as ET

response = requests.get(f"{base_url}/sitemap.xml")
root = ET.fromstring(response.content)
urls = [loc.text for loc in root.iter('{http://www.sitemaps.org/schemas/sitemap/0.9}loc')]
```

**Fallback**: If sitemap unavailable, crawl from homepage following internal links.

---

### 6. Rate Limiting and Retry Strategy

**Decision**: Exponential backoff with jitter for both Cohere and Qdrant APIs.

**Rationale**:
- Cohere free tier: 1000 calls/minute, 10000 calls/month
- Batch embeddings (up to 96 texts per call) to minimize API calls
- Exponential backoff prevents thundering herd on rate limit recovery

**Implementation**:
```python
import time
import random

def retry_with_backoff(func, max_retries=5):
    for attempt in range(max_retries):
        try:
            return func()
        except RateLimitError:
            wait = (2 ** attempt) + random.uniform(0, 1)
            time.sleep(wait)
    raise Exception("Max retries exceeded")
```

**Batch Size**: 96 texts per Cohere embed call (API limit)

---

### 7. Deduplication Strategy (FR-013, FR-014)

**Decision**: Content hash comparison before embedding + Qdrant scroll query for existing URLs.

**Rationale**:
- Content hash (SHA-256) detects identical chunk text regardless of source
- URL + chunk_index lookup enables idempotent re-runs
- Check existing vectors before insertion to prevent duplicates

**Implementation**:
```python
import hashlib

def content_hash(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()

# Before inserting, check if URL+chunk_index exists
existing = client.scroll(
    collection_name="book_embeddings",
    scroll_filter=Filter(must=[
        FieldCondition(key="url", match=MatchValue(value=url)),
        FieldCondition(key="chunk_index", match=MatchValue(value=chunk_idx))
    ]),
    limit=1
)
if not existing[0]:  # No existing record
    client.upsert(...)
```

---

### 8. Logging Configuration

**Decision**: Python logging module with configurable levels (INFO default, DEBUG optional).

**Rationale**:
- Standard library, no additional dependencies
- Structured log format for metrics tracking
- Never log API keys or credentials (FR-011, SC-006)

**Log Levels** (per FR-012):
- INFO: Progress (pages discovered, chunks created, embeddings generated)
- ERROR: Failures (network errors, API errors, validation failures)
- DEBUG: Detailed traces (individual chunk processing, API responses)

**Metrics to Log**:
- Pages discovered from sitemap
- Pages successfully scraped
- Chunks created per page
- Embeddings generated
- Storage successes/failures
- Validation pass/fail counts

---

## Resolved Clarifications

| Item | Resolution |
|------|------------|
| Embedding model | embed-english-v3.0 (1024 dimensions) |
| Token counting | Cohere tokenizer API |
| Chunk size handling | Split chunks >512 tokens before embedding |
| Deduplication | Content hash + URL/index lookup |
| HTML extraction | BeautifulSoup with Docusaurus selectors |
| Rate limiting | Exponential backoff, batch processing |

## External References

- [Cohere Embed API Documentation](https://docs.cohere.com/reference/embed)
- [Qdrant Python Client](https://qdrant.tech/documentation/quick-start/)
- [Docusaurus Generated Files](https://docusaurus.io/docs/advanced/routing#generated-files)
