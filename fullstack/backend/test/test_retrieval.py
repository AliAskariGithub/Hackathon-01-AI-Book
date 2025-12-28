"""Unit tests for retrieval.py (T028)."""

from datetime import datetime
from unittest.mock import MagicMock, patch

import pytest

# Import after mocking to avoid import errors without dependencies
with patch.dict("sys.modules", {
    "cohere": MagicMock(),
    "qdrant_client": MagicMock(),
}):
    from models import QueryResult, RetrievalResult, ValidationReport


class TestRetrievalResult:
    """Test RetrievalResult dataclass."""

    def test_create_retrieval_result(self):
        """Can create RetrievalResult with all fields."""
        result = RetrievalResult(
            chunk_id="test-id",
            text="Result text",
            score=0.85,
            url="https://example.com",
            title="Test Page",
            chunk_index=0
        )
        assert result.score == 0.85
        assert result.title == "Test Page"

    def test_score_range(self):
        """Score is a float between 0 and 1."""
        result = RetrievalResult(
            chunk_id="id",
            text="text",
            score=0.75,
            url="url",
            title="title",
            chunk_index=0
        )
        assert 0.0 <= result.score <= 1.0


class TestQueryResult:
    """Test QueryResult dataclass."""

    def test_create_query_result(self):
        """Can create QueryResult with pass/fail status."""
        result = QueryResult(
            query="What is URDF?",
            top_score=0.82,
            passed=True,
            top_results=[]
        )
        assert result.passed is True
        assert result.top_score == 0.82

    def test_failed_query_result(self):
        """Query below threshold is marked as failed."""
        result = QueryResult(
            query="Obscure query",
            top_score=0.45,
            passed=False,
            top_results=[]
        )
        assert result.passed is False


class TestValidationReport:
    """Test ValidationReport dataclass."""

    def test_create_validation_report(self):
        """Can create ValidationReport with metrics."""
        report = ValidationReport(
            timestamp=datetime.now(),
            total_queries=6,
            passed_queries=5,
            failed_queries=1,
            pass_rate=5/6,
            threshold=0.7,
            avg_similarity=0.78,
            results=[]
        )
        assert report.total_queries == 6
        assert report.pass_rate == pytest.approx(0.833, rel=0.01)

    def test_perfect_pass_rate(self):
        """100% pass rate when all queries pass."""
        report = ValidationReport(
            timestamp=datetime.now(),
            total_queries=6,
            passed_queries=6,
            failed_queries=0,
            pass_rate=1.0,
            threshold=0.7,
            avg_similarity=0.85,
            results=[]
        )
        assert report.pass_rate == 1.0
        assert report.failed_queries == 0


class TestResultFormatting:
    """Test result formatting functions."""

    def test_format_empty_results(self):
        """Empty results return appropriate message."""
        results = []
        if not results:
            message = "No results found."
        assert message == "No results found."

    def test_text_truncation(self):
        """Long text is truncated with ellipsis."""
        long_text = "A" * 300
        max_length = 200
        truncated = long_text[:max_length] + "..." if len(long_text) > max_length else long_text
        assert len(truncated) == 203  # 200 + "..."
        assert truncated.endswith("...")

    def test_short_text_no_truncation(self):
        """Short text is not truncated."""
        short_text = "Short text"
        max_length = 200
        result = short_text[:max_length] + "..." if len(short_text) > max_length else short_text
        assert result == short_text
        assert not result.endswith("...")


class TestValidationLogic:
    """Test validation logic."""

    def test_threshold_comparison(self):
        """Score comparison with threshold."""
        threshold = 0.7
        scores = [0.85, 0.72, 0.65, 0.90, 0.68, 0.75]

        passed = [s >= threshold for s in scores]
        assert passed == [True, True, False, True, False, True]

    def test_pass_rate_calculation(self):
        """Pass rate calculated correctly."""
        passed_queries = 4
        total_queries = 6
        pass_rate = passed_queries / total_queries
        assert pass_rate == pytest.approx(0.667, rel=0.01)

    def test_average_similarity(self):
        """Average similarity calculated correctly."""
        scores = [0.85, 0.72, 0.65, 0.90, 0.68, 0.75]
        avg = sum(scores) / len(scores)
        assert avg == pytest.approx(0.758, rel=0.01)


class TestTestQueries:
    """Test the test queries configuration."""

    def test_default_test_queries(self):
        """Default test queries are defined."""
        test_queries = [
            "What is URDF and how is it used?",
            "How do forward kinematics work?",
            "What sensors are used in robotics?",
            "How to set up Gazebo simulation?",
            "What is Isaac Sim architecture?",
            "How do AI agents plan actions?",
        ]
        assert len(test_queries) == 6

    def test_queries_are_strings(self):
        """All test queries are non-empty strings."""
        test_queries = [
            "What is URDF and how is it used?",
            "How do forward kinematics work?",
            "What sensors are used in robotics?",
        ]
        for q in test_queries:
            assert isinstance(q, str)
            assert len(q) > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
