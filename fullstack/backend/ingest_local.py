"""Local ingestion script that reads from markdown files directly."""

import os
import glob
import hashlib
import logging
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import List

import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, PointStruct, VectorParams, PayloadSchemaType

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


def read_local_markdown_files(docs_dir: str) -> List[ContentPage]:
    """Read all markdown files from local docs directory."""
    logger.info(f"Reading markdown files from {docs_dir}")

    pages = []
    md_files = glob.glob(os.path.join(docs_dir, "**/*.md"), recursive=True)

    logger.info(f"Found {len(md_files)} markdown files")

    for file_path in md_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Skip if content is too short
            if len(content) < 50:
                logger.warning(f"Skipping {file_path} - too short")
                continue

            # Extract title from first heading or filename
            title = Path(file_path).stem.replace('-', ' ').title()
            lines = content.split('\n')
            for line in lines:
                if line.startswith('# '):
                    title = line.replace('# ', '').strip()
                    break

            # Create relative URL path
            rel_path = os.path.relpath(file_path, docs_dir)
            url = f"/docs/{rel_path.replace(os.sep, '/').replace('.md', '')}"

            pages.append(ContentPage(
                url=url,
                title=title,
                raw_content=content,
                scraped_at=datetime.now()
            ))

            logger.debug(f"Loaded: {title} ({url})")

        except Exception as e:
            logger.warning(f"Failed to read {file_path}: {e}")
            continue

    logger.info(f"Successfully loaded {len(pages)} pages")
    return pages


def count_tokens(text: str, co: cohere.Client) -> int:
    """Count tokens using Cohere tokenizer."""
    try:
        response = co.tokenize(text=text, model=COHERE_MODEL)
        return len(response.tokens)
    except Exception as e:
        logger.warning(f"Token counting failed, estimating: {e}")
        return int(len(text.split()) * 1.3)


def chunk_content(
    page: ContentPage,
    co: cohere.Client,
    target_tokens: int = 750,
    overlap_tokens: int = 100
) -> List[ContentChunk]:
    """Split page content into overlapping chunks."""
    text = page.raw_content
    chunks = []

    # Split by paragraphs first
    paragraphs = [p.strip() for p in text.split("\n\n") if p.strip()]

    current_chunk = ""
    current_tokens = 0
    chunk_index = 0

    for para in paragraphs:
        para_tokens = count_tokens(para, co)

        if para_tokens > target_tokens:
            if current_chunk:
                chunks.append(_create_chunk(current_chunk, chunk_index, page, co))
                chunk_index += 1
                current_chunk = ""
                current_tokens = 0

            sentences = para.replace(". ", ".\n").split("\n")
            for sentence in sentences:
                sent_tokens = count_tokens(sentence, co)
                if current_tokens + sent_tokens > target_tokens and current_chunk:
                    chunks.append(_create_chunk(current_chunk, chunk_index, page, co))
                    chunk_index += 1
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
                overlap_text = current_chunk[-overlap_tokens * 4:] if len(current_chunk) > overlap_tokens * 4 else ""
                current_chunk = overlap_text + "\n\n" + para
                current_tokens = count_tokens(current_chunk, co)
            else:
                current_chunk += "\n\n" + para if current_chunk else para
                current_tokens += para_tokens

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
    """Generate embeddings with batch processing."""
    embeddings = []
    total_batches = (len(chunks) + batch_size - 1) // batch_size

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        batch_num = i // batch_size + 1
        logger.info(f"Generating embeddings batch {batch_num}/{total_batches}")

        texts = [chunk.text for chunk in batch]

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
        except Exception as e:
            logger.error(f"Embedding batch {batch_num} failed: {e}")
            raise

    logger.info(f"Generated {len(embeddings)} embeddings")
    return embeddings


def init_collection(client: QdrantClient, collection_name: str = COLLECTION_NAME) -> None:
    """Create Qdrant collection if it doesn't exist."""
    collections = client.get_collections().collections
    exists = any(c.name == collection_name for c in collections)

    if not exists:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE)
        )
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
    """Store vectors with deduplication check."""
    chunk_map = {c.id: c for c in chunks}

    # Get existing hashes
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

    # Build points list
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
        batch_size = 50
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            client.upsert(collection_name=collection_name, points=batch)
            stored += len(batch)
            logger.info(f"Stored batch {i // batch_size + 1}: {len(batch)} vectors")

    logger.info(f"Stored {stored} vectors, skipped {skipped} duplicates")
    return stored, skipped


def ingest_local_book(docs_dir: str, collection_name: str = COLLECTION_NAME) -> IngestionResult:
    """Orchestrate local ingestion pipeline."""
    start_time = time.time()
    errors = []

    # Initialize clients
    co = cohere.Client(os.getenv("COHERE_API_KEY"))
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Initialize collection
    init_collection(qdrant, collection_name)

    # Read local files
    pages = read_local_markdown_files(docs_dir)

    if not pages:
        return IngestionResult(
            success=False,
            pages_discovered=0,
            pages_scraped=0,
            chunks_created=0,
            embeddings_generated=0,
            vectors_stored=0,
            duplicates_skipped=0,
            errors=["No markdown files found"],
            duration_seconds=time.time() - start_time
        )

    # Chunk content
    all_chunks = []
    for page in pages:
        chunks = chunk_content(page, co)
        all_chunks.extend(chunks)

    logger.info(f"Created {len(all_chunks)} chunks")

    # Generate embeddings
    try:
        embeddings = generate_embeddings(all_chunks, co)
    except Exception as e:
        errors.append(f"Embedding generation failed: {e}")
        return IngestionResult(
            success=False,
            pages_discovered=len(pages),
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
        pages_discovered=len(pages),
        pages_scraped=len(pages),
        chunks_created=len(all_chunks),
        embeddings_generated=len(embeddings),
        vectors_stored=stored,
        duplicates_skipped=skipped,
        errors=errors,
        duration_seconds=time.time() - start_time
    )


if __name__ == "__main__":
    docs_dir = "C:/Hackathons/hackathon-ai-book/fullstack/frontend-book/docs"

    logger.info(f"Starting local ingestion from {docs_dir}")
    result = ingest_local_book(docs_dir)

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
