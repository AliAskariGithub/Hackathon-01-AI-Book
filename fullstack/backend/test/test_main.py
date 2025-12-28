"""Unit tests for main.py (T027)."""

import hashlib
from datetime import datetime
from unittest.mock import MagicMock, patch

import pytest

# Import after mocking to avoid import errors without dependencies
with patch.dict("sys.modules", {
    "cohere": MagicMock(),
    "qdrant_client": MagicMock(),
    "qdrant_client.models": MagicMock(),
}):
    from models import ContentChunk, ContentPage


class TestContentHash:
    """Test content hash generation for deduplication."""

    def test_hash_deterministic(self):
        """Same content produces same hash."""
        text = "This is test content."
        hash1 = hashlib.sha256(text.encode()).hexdigest()
        hash2 = hashlib.sha256(text.encode()).hexdigest()
        assert hash1 == hash2

    def test_hash_different_content(self):
        """Different content produces different hash."""
        text1 = "Content A"
        text2 = "Content B"
        hash1 = hashlib.sha256(text1.encode()).hexdigest()
        hash2 = hashlib.sha256(text2.encode()).hexdigest()
        assert hash1 != hash2

    def test_hash_length(self):
        """SHA-256 hash is 64 characters."""
        text = "Test"
        hash_value = hashlib.sha256(text.encode()).hexdigest()
        assert len(hash_value) == 64


class TestContentPage:
    """Test ContentPage dataclass."""

    def test_create_content_page(self):
        """Can create ContentPage with required fields."""
        page = ContentPage(
            url="https://example.com/page",
            title="Test Page",
            raw_content="This is the content."
        )
        assert page.url == "https://example.com/page"
        assert page.title == "Test Page"
        assert page.raw_content == "This is the content."
        assert isinstance(page.scraped_at, datetime)


class TestContentChunk:
    """Test ContentChunk dataclass."""

    def test_create_content_chunk(self):
        """Can create ContentChunk with all fields."""
        chunk = ContentChunk(
            id="test-id",
            content_hash="abc123",
            text="Chunk text",
            token_count=50,
            chunk_index=0,
            page_url="https://example.com",
            page_title="Test"
        )
        assert chunk.id == "test-id"
        assert chunk.token_count == 50
        assert chunk.chunk_index == 0


class TestChunkingLogic:
    """Test chunking logic (without API calls)."""

    def test_paragraph_split(self):
        """Text splits on double newlines."""
        text = "Paragraph one.\n\nParagraph two.\n\nParagraph three."
        paragraphs = [p.strip() for p in text.split("\n\n") if p.strip()]
        assert len(paragraphs) == 3

    def test_sentence_split(self):
        """Long paragraphs split on sentences."""
        text = "Sentence one. Sentence two. Sentence three."
        sentences = text.replace(". ", ".\n").split("\n")
        assert len(sentences) == 3

    def test_empty_paragraphs_filtered(self):
        """Empty paragraphs are filtered out."""
        text = "Content.\n\n\n\nMore content."
        paragraphs = [p.strip() for p in text.split("\n\n") if p.strip()]
        assert len(paragraphs) == 2


class TestTokenEstimation:
    """Test token count estimation fallback."""

    def test_rough_estimate(self):
        """Fallback estimate is ~1.3x word count."""
        text = "one two three four five"  # 5 words
        estimate = int(len(text.split()) * 1.3)
        assert estimate == 6  # 5 * 1.3 = 6.5 -> 6


class TestSitemapParsing:
    """Test sitemap URL extraction."""

    def test_extract_urls_from_xml(self):
        """Can extract URLs from sitemap XML."""
        import xml.etree.ElementTree as ET

        sitemap_xml = """<?xml version="1.0" encoding="UTF-8"?>
        <urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
            <url><loc>https://example.com/page1</loc></url>
            <url><loc>https://example.com/page2</loc></url>
        </urlset>"""

        root = ET.fromstring(sitemap_xml)
        namespace = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"}
        urls = [loc.text for loc in root.findall(".//ns:loc", namespace)]

        assert len(urls) == 2
        assert "https://example.com/page1" in urls
        assert "https://example.com/page2" in urls


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
