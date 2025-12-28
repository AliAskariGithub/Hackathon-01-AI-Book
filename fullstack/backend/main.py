"""RAG pipeline for book content ingestion and embedding."""

import argparse
import hashlib
import logging
import os
import random
import time
import uuid
import xml.etree.ElementTree as ET
from datetime import datetime
from typing import List, Optional

import cohere
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, FieldCondition, Filter, MatchValue, PointStruct, VectorParams, PayloadSchemaType

from models import ContentChunk, ContentPage, IngestionResult, VectorEmbedding

# Load environment variables
load_dotenv()

# Configure logging
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL),
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Configuration
BATCH_SIZE = int(os.getenv("BATCH_SIZE", "96"))
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "book_embeddings")
COHERE_MODEL = "embed-english-v3.0"
VECTOR_SIZE = 1024


def scrape_sitemap(base_url: str) -> List[str]:
    """Fetch and parse sitemap.xml to extract page URLs (T013)."""
    sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"
    logger.info(f"Fetching sitemap from {sitemap_url}")

    try:
        response = requests.get(sitemap_url, timeout=30)
        response.raise_for_status()
    except requests.RequestException as e:
        logger.error(f"Failed to fetch sitemap: {e}")
        raise

    root = ET.fromstring(response.content)
    namespace = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"}

    urls = []
    for loc in root.findall(".//ns:loc", namespace):
        if loc.text:
            urls.append(loc.text)

    logger.info(f"Discovered {len(urls)} pages from sitemap")
    return urls


def scrape_page(url: str) -> Optional[ContentPage]:
    """Fetch and extract clean text from a single page (T014)."""
    logger.debug(f"Scraping page: {url}")

    try:
        response = requests.get(url, timeout=30)
        response.raise_for_status()
    except requests.RequestException as e:
        logger.warning(f"Failed to fetch page {url}: {e}")
        return None

    soup = BeautifulSoup(response.content, "html.parser")

    # Extract title
    title_tag = soup.find("title")
    title = title_tag.get_text(strip=True) if title_tag else "Untitled"

    # Extract main content (Docusaurus structure)
    content = soup.select_one("article") or soup.select_one(".markdown") or soup.select_one("main")

    if not content:
        logger.warning(f"No content found for {url}")
        return None

    # Remove unwanted elements
    for selector in ["nav", "footer", ".table-of-contents", ".pagination-nav", "script", "style"]:
        for element in content.select(selector):
            element.decompose()

    text = content.get_text(separator="\n", strip=True)

    if len(text) < 50:
        logger.warning(f"Insufficient content for {url} ({len(text)} chars)")
        return None

    return ContentPage(
        url=url,
        title=title,
        raw_content=text,
        scraped_at=datetime.now()
    )


def count_tokens(text: str, co: cohere.Client) -> int:
    """Count tokens using Cohere tokenizer (T015)."""
    try:
        response = co.tokenize(text=text, model=COHERE_MODEL)
        return len(response.tokens)
    except Exception as e:
        logger.warning(f"Token counting failed, estimating: {e}")
        return int(len(text.split()) * 1.3)  # Rough estimate


def chunk_content(
    page: ContentPage,
    co: cohere.Client,
    target_tokens: int = 750,
    overlap_tokens: int = 100
) -> List[ContentChunk]:
    """Split page content into overlapping chunks (T016)."""
    text = page.raw_content
    chunks = []

    # Split by paragraphs first
    paragraphs = [p.strip() for p in text.split("\n\n") if p.strip()]

    current_chunk = ""
    current_tokens = 0
    chunk_index = 0

    for para in paragraphs:
        para_tokens = count_tokens(para, co)

        # If single paragraph exceeds target, split it
        if para_tokens > target_tokens:
            if current_chunk:
                chunks.append(_create_chunk(current_chunk, chunk_index, page, co))
                chunk_index += 1
                current_chunk = ""
                current_tokens = 0

            # Split long paragraph by sentences
            sentences = para.replace(". ", ".\n").split("\n")
            for sentence in sentences:
                sent_tokens = count_tokens(sentence, co)
                if current_tokens + sent_tokens > target_tokens and current_chunk:
                    chunks.append(_create_chunk(current_chunk, chunk_index, page, co))
                    chunk_index += 1
                    # Keep overlap
                    overlap_text = current_chunk[-overlap_tokens * 4:] if len(current_chunk) > overlap_tokens * 4 else ""
                    current_chunk = overlap_text + " " + sentence
                    current_tokens = count_tokens(current_chunk, co)
                else:
                    current_chunk += " " + sentence if current_chunk else sentence
                    current_tokens += sent_tokens
        else:
            if current_tokens + para_tokens > target_tokens and current_chunk:
                chunks.append(_create_chunk(current_chunk, chunk_index, page, co))
                chunk_index += 1
                # Keep overlap
                overlap_text = current_chunk[-overlap_tokens * 4:] if len(current_chunk) > overlap_tokens * 4 else ""
                current_chunk = overlap_text + "\n\n" + para
                current_tokens = count_tokens(current_chunk, co)
            else:
                current_chunk += "\n\n" + para if current_chunk else para
                current_tokens += para_tokens

    # Add final chunk
    if current_chunk:
        chunks.append(_create_chunk(current_chunk, chunk_index, page, co))

    logger.debug(f"Created {len(chunks)} chunks for {page.url}")
    return chunks


def _create_chunk(text: str, index: int, page: ContentPage, co: cohere.Client) -> ContentChunk:
    """Helper to create a ContentChunk with hash."""
    return ContentChunk(
        id=str(uuid.uuid4()),
        content_hash=hashlib.sha256(text.encode()).hexdigest(),
        text=text.strip(),
        token_count=count_tokens(text, co),
        chunk_index=index,
        page_url=page.url,
        page_title=page.title
    )


def generate_embeddings(
    chunks: List[ContentChunk],
    co: cohere.Client,
    batch_size: int = BATCH_SIZE
) -> List[VectorEmbedding]:
    """Generate embeddings with batch processing and retry logic (T017)."""
    embeddings = []
    total_batches = (len(chunks) + batch_size - 1) // batch_size

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        batch_num = i // batch_size + 1
        logger.info(f"Generating embeddings batch {batch_num}/{total_batches}")

        texts = [chunk.text for chunk in batch]

        # Retry with exponential backoff
        for attempt in range(5):
            try:
                response = co.embed(
                    texts=texts,
                    model=COHERE_MODEL,
                    input_type="search_document"
                )

                for chunk, vector in zip(batch, response.embeddings):
                    embeddings.append(VectorEmbedding(
                        chunk_id=chunk.id,
                        vector=vector,
                        model=COHERE_MODEL,
                        input_type="search_document"
                    ))
                break

            except Exception as e:
                wait = (2 ** attempt) + random.uniform(0, 1)
                logger.warning(f"Embedding failed (attempt {attempt + 1}): {e}, retrying in {wait:.1f}s")
                time.sleep(wait)
                if attempt == 4:
                    logger.error(f"Failed to generate embeddings after 5 attempts")
                    raise

    logger.info(f"Generated {len(embeddings)} embeddings")
    return embeddings


def init_collection(client: QdrantClient, collection_name: str = COLLECTION_NAME) -> None:
    """Create Qdrant collection if it doesn't exist (T018)."""
    collections = client.get_collections().collections
    exists = any(c.name == collection_name for c in collections)

    if not exists:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE)
        )
        # Create payload index for deduplication queries
        client.create_payload_index(
            collection_name=collection_name,
            field_name="content_hash",
            field_schema=PayloadSchemaType.KEYWORD
        )
        logger.info(f"Created collection: {collection_name}")
    else:
        logger.info(f"Collection exists: {collection_name}")


def store_vectors(
    embeddings: List[VectorEmbedding],
    chunks: List[ContentChunk],
    client: QdrantClient,
    collection_name: str = COLLECTION_NAME
) -> tuple[int, int]:
    """Store vectors with deduplication check (T019)."""
    chunk_map = {c.id: c for c in chunks}

    # Get all existing content hashes in one query (FR-014 batch check)
    existing_hashes = set()
    try:
        offset = None
        while True:
            result = client.scroll(
                collection_name=collection_name,
                limit=100,
                offset=offset,
                with_payload=["content_hash"]
            )
            points, offset = result
            for p in points:
                if p.payload and "content_hash" in p.payload:
                    existing_hashes.add(p.payload["content_hash"])
            if offset is None:
                break
        logger.info(f"Found {len(existing_hashes)} existing vectors")
    except Exception as e:
        logger.warning(f"Could not fetch existing hashes: {e}")

    # Build points list, skipping duplicates
    points = []
    skipped = 0
    for emb in embeddings:
        chunk = chunk_map.get(emb.chunk_id)
        if not chunk:
            continue

        if chunk.content_hash in existing_hashes:
            logger.debug(f"Skipping duplicate: {chunk.content_hash[:16]}...")
            skipped += 1
            continue

        points.append(PointStruct(
            id=chunk.id,
            vector=emb.vector,
            payload={
                "url": chunk.page_url,
                "title": chunk.page_title,
                "text": chunk.text,
                "chunk_index": chunk.chunk_index,
                "content_hash": chunk.content_hash,
                "token_count": chunk.token_count,
                "created_at": datetime.now().isoformat()
            }
        ))

    stored = 0
    if points:
        # Upsert in batches to avoid timeout
        batch_size = 50
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            client.upsert(collection_name=collection_name, points=batch)
            stored += len(batch)
            logger.info(f"Stored batch {i // batch_size + 1}: {len(batch)} vectors")

    logger.info(f"Stored {stored} vectors, skipped {skipped} duplicates")
    return stored, skipped


def ingest_book(
    base_url: str,
    collection_name: str = COLLECTION_NAME,
    dry_run: bool = False
) -> IngestionResult:
    """Orchestrate full ingestion pipeline (T020)."""
    start_time = time.time()
    errors = []

    # Initialize clients
    co = cohere.Client(os.getenv("COHERE_API_KEY"))
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Initialize collection
    if not dry_run:
        init_collection(qdrant, collection_name)

    # Scrape sitemap
    try:
        urls = scrape_sitemap(base_url)
    except Exception as e:
        return IngestionResult(
            success=False,
            pages_discovered=0,
            pages_scraped=0,
            chunks_created=0,
            embeddings_generated=0,
            vectors_stored=0,
            duplicates_skipped=0,
            errors=[str(e)],
            duration_seconds=time.time() - start_time
        )

    # Scrape pages
    pages = []
    for url in urls:
        page = scrape_page(url)
        if page:
            pages.append(page)

    logger.info(f"Scraped {len(pages)}/{len(urls)} pages")

    # Chunk content
    all_chunks = []
    for page in pages:
        chunks = chunk_content(page, co)
        all_chunks.extend(chunks)

    logger.info(f"Created {len(all_chunks)} chunks")

    if dry_run:
        return IngestionResult(
            success=True,
            pages_discovered=len(urls),
            pages_scraped=len(pages),
            chunks_created=len(all_chunks),
            embeddings_generated=0,
            vectors_stored=0,
            duplicates_skipped=0,
            duration_seconds=time.time() - start_time
        )

    # Generate embeddings
    try:
        embeddings = generate_embeddings(all_chunks, co)
    except Exception as e:
        errors.append(f"Embedding generation failed: {e}")
        return IngestionResult(
            success=False,
            pages_discovered=len(urls),
            pages_scraped=len(pages),
            chunks_created=len(all_chunks),
            embeddings_generated=0,
            vectors_stored=0,
            duplicates_skipped=0,
            errors=errors,
            duration_seconds=time.time() - start_time
        )

    # Store vectors
    stored, skipped = store_vectors(embeddings, all_chunks, qdrant, collection_name)

    return IngestionResult(
        success=True,
        pages_discovered=len(urls),
        pages_scraped=len(pages),
        chunks_created=len(all_chunks),
        embeddings_generated=len(embeddings),
        vectors_stored=stored,
        duplicates_skipped=skipped,
        errors=errors,
        duration_seconds=time.time() - start_time
    )


def main():
    """CLI interface (T021)."""
    parser = argparse.ArgumentParser(description="RAG book content ingestion")
    parser.add_argument("--url", required=True, help="Base URL of the deployed book")
    parser.add_argument("--collection", default=COLLECTION_NAME, help="Qdrant collection name")
    parser.add_argument("--dry-run", action="store_true", help="Scrape and chunk without storing")

    args = parser.parse_args()

    logger.info(f"Starting ingestion for {args.url}")
    result = ingest_book(args.url, args.collection, args.dry_run)

    print("\n" + "=" * 50)
    print("Ingestion Complete" if result.success else "Ingestion Failed")
    print("=" * 50)
    print(f"Pages discovered: {result.pages_discovered}")
    print(f"Pages scraped: {result.pages_scraped}")
    print(f"Chunks created: {result.chunks_created}")
    print(f"Embeddings generated: {result.embeddings_generated}")
    print(f"Vectors stored: {result.vectors_stored}")
    print(f"Duplicates skipped: {result.duplicates_skipped}")
    print(f"Duration: {result.duration_seconds:.1f}s")

    if result.errors:
        print(f"\nErrors:")
        for err in result.errors:
            print(f"  - {err}")

    return 0 if result.success else 1


if __name__ == "__main__":
    exit(main())
