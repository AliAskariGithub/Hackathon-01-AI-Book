"""Tests for RAG Agent (T046-T051)."""

import os
import tempfile
import pytest
from unittest.mock import patch, MagicMock, AsyncMock

# Import agent module components
from agent import (
    count_tokens,
    extract_citations,
    handle_error,
    create_agent,
    BUDGET_SYSTEM,
    BUDGET_CONTEXT,
    BUDGET_HISTORY,
    BUDGET_RESPONSE,
    BUDGET_TOTAL
)
from models import (
    AgentConfig,
    AgentState,
    Citation,
    Conversation,
    Message,
    RetrievalContext,
    RetrievalResult,
    RetrievalError,
    GenerationError,
    ConfigurationError,
    ErrorResponse
)


# =============================================================================
# T047: Unit tests for count_tokens()
# =============================================================================
class TestCountTokens:
    """Tests for the count_tokens function."""

    def test_empty_string_returns_zero(self):
        """Empty string should return 0 tokens."""
        assert count_tokens("") == 0

    def test_none_returns_zero(self):
        """None-like empty input should return 0."""
        assert count_tokens("") == 0

    def test_hello_world(self):
        """Known string should return expected token count."""
        # "Hello world!" typically tokenizes to 3 tokens
        result = count_tokens("Hello world!")
        assert result == 3

    def test_longer_sentence(self):
        """Longer sentence should return reasonable token count."""
        text = "What is URDF and how is it used in robotics?"
        result = count_tokens(text)
        # Should be around 10-15 tokens
        assert 8 <= result <= 20

    def test_unicode_text(self):
        """Unicode text should be handled correctly."""
        text = "Héllo wörld! 你好"
        result = count_tokens(text)
        assert result > 0


# =============================================================================
# T048: Unit tests for extract_citations()
# =============================================================================
class TestExtractCitations:
    """Tests for the extract_citations function."""

    def test_extracts_single_citation(self):
        """Should extract a single citation from response."""
        response = "URDF is a format [Source: URDF Basics](https://example.com/urdf)."
        context = RetrievalContext(
            query="test",
            results=[
                RetrievalResult(
                    chunk_id="1",
                    text="test",
                    score=0.8,
                    url="https://example.com/urdf",
                    title="URDF Basics",
                    chunk_index=0
                )
            ]
        )

        citations = extract_citations(response, context)

        assert len(citations) == 1
        assert citations[0].title == "URDF Basics"
        assert citations[0].url == "https://example.com/urdf"

    def test_extracts_multiple_citations(self):
        """Should extract multiple citations from response."""
        response = """
        URDF is explained in [Source: URDF Basics](https://example.com/urdf).
        Also see [Source: Robot Models](https://example.com/robot).
        """
        context = RetrievalContext(
            query="test",
            results=[
                RetrievalResult(
                    chunk_id="1",
                    text="test",
                    score=0.8,
                    url="https://example.com/urdf",
                    title="URDF Basics",
                    chunk_index=0
                ),
                RetrievalResult(
                    chunk_id="2",
                    text="test",
                    score=0.7,
                    url="https://example.com/robot",
                    title="Robot Models",
                    chunk_index=0
                )
            ]
        )

        citations = extract_citations(response, context)

        assert len(citations) == 2

    def test_no_citations_returns_empty_list(self):
        """Should return empty list when no citations found."""
        response = "This response has no citations."
        context = RetrievalContext(query="test", results=[])

        citations = extract_citations(response, context)

        assert len(citations) == 0

    def test_ignores_invalid_urls(self):
        """Should ignore citations with URLs not in context."""
        response = "See [Source: Unknown](https://unknown.com/page)."
        context = RetrievalContext(
            query="test",
            results=[
                RetrievalResult(
                    chunk_id="1",
                    text="test",
                    score=0.8,
                    url="https://example.com/valid",
                    title="Valid",
                    chunk_index=0
                )
            ]
        )

        citations = extract_citations(response, context)

        assert len(citations) == 0


# =============================================================================
# T049: Test conversation reset
# =============================================================================
class TestConversationReset:
    """Tests for conversation reset functionality."""

    def test_conversation_resets_on_clear(self):
        """Verify conversation state doesn't persist across clears."""
        conversation = Conversation()

        # Simulate conversation
        msg1 = Message(role="user", content="test", token_count=5)
        msg2 = Message(role="assistant", content="response", token_count=10)
        conversation.add_message(msg1)
        conversation.add_message(msg2)

        assert len(conversation.messages) == 2
        assert conversation.total_tokens == 15

        # Clear conversation
        conversation.clear()

        assert len(conversation.messages) == 0
        assert conversation.total_tokens == 0

    def test_new_conversation_is_empty(self):
        """New conversation should be empty."""
        conversation = Conversation()

        assert len(conversation.messages) == 0
        assert conversation.total_tokens == 0


# =============================================================================
# T050: Test no file persistence
# =============================================================================
class TestNoFilePersistence:
    """Tests to verify no unintended file writes."""

    def test_conversation_operations_create_no_files(self):
        """Verify no files created during conversation operations."""
        with tempfile.TemporaryDirectory() as tmpdir:
            original_cwd = os.getcwd()
            os.chdir(tmpdir)

            try:
                files_before = set(os.listdir(tmpdir))

                # Perform conversation operations
                conversation = Conversation()
                conversation.add_message(Message(role="user", content="test"))
                conversation.add_message(Message(role="assistant", content="response"))
                conversation.trim_to_budget(100)
                conversation.clear()

                files_after = set(os.listdir(tmpdir))
                assert files_before == files_after, "Unexpected files created"
            finally:
                os.chdir(original_cwd)


# =============================================================================
# Additional tests for error handling
# =============================================================================
class TestErrorHandling:
    """Tests for error handling functions."""

    def test_handle_retrieval_error(self):
        """RetrievalError should produce recoverable error."""
        error = RetrievalError("Search failed")
        response = handle_error(error)

        assert response.error_type == "retrieval"
        assert response.recoverable is True

    def test_handle_generation_error(self):
        """GenerationError should produce recoverable error."""
        error = GenerationError("LLM failed")
        response = handle_error(error)

        assert response.error_type == "generation"
        assert response.recoverable is True

    def test_handle_configuration_error(self):
        """ConfigurationError should produce non-recoverable error."""
        error = ConfigurationError("Missing API key")
        response = handle_error(error)

        assert response.error_type == "config"
        assert response.recoverable is False

    def test_handle_unknown_error(self):
        """Unknown errors should be handled gracefully."""
        error = Exception("Unknown error")
        response = handle_error(error)

        assert response.error_type == "unknown"
        assert response.recoverable is True


# =============================================================================
# Tests for budget constants
# =============================================================================
class TestBudgetConstants:
    """Tests for budget constant values."""

    def test_budget_total_is_sum_of_parts(self):
        """Budget total should be sum of component budgets."""
        expected = BUDGET_SYSTEM + BUDGET_CONTEXT + BUDGET_HISTORY + BUDGET_RESPONSE
        # Note: BUDGET_TOTAL is slightly less for safety margin
        assert BUDGET_TOTAL <= expected

    def test_budget_total_under_context_window(self):
        """Budget total should be under 8192 context window."""
        assert BUDGET_TOTAL < 8192


# =============================================================================
# Tests for conversation trimming
# =============================================================================
class TestConversationTrimming:
    """Tests for conversation budget management."""

    def test_trim_removes_oldest_messages(self):
        """Trim should remove oldest non-system messages first."""
        conversation = Conversation()

        # Add messages with known token counts
        for i in range(5):
            msg = Message(role="user", content=f"msg{i}", token_count=100)
            conversation.add_message(msg)

        assert conversation.total_tokens == 500

        # Trim to 300 tokens (should remove 2 messages)
        conversation.trim_to_budget(300)

        assert conversation.total_tokens == 300
        assert len(conversation.messages) == 3

    def test_trim_preserves_system_message(self):
        """Trim should preserve system message at index 0."""
        conversation = Conversation()

        # Add system message first
        system_msg = Message(role="system", content="system prompt", token_count=100)
        conversation.add_message(system_msg)

        # Add user messages
        for i in range(3):
            msg = Message(role="user", content=f"msg{i}", token_count=100)
            conversation.add_message(msg)

        assert conversation.total_tokens == 400

        # Trim to 200 tokens
        conversation.trim_to_budget(200)

        # System message should still be first
        assert conversation.messages[0].role == "system"
