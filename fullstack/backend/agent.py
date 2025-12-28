"""RAG Agent for conversational Q&A over book content.

This module implements a conversational AI agent that:
- Retrieves relevant content from a Qdrant vector database
- Generates grounded responses using OpenRouter LLMs
- Maintains conversation context across turns
- Provides citations for all answers
"""

import asyncio
import logging
import os
import re
from typing import List, Optional

import tiktoken
from dotenv import load_dotenv
from openai import AsyncOpenAI, RateLimitError, APIConnectionError
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

from models import (
    AgentConfig, AgentState, AgentError, Citation, ConfigurationError,
    Conversation, ErrorResponse, GenerationError, Message, RetrievalContext,
    RetrievalError, RetrievalResult
)
from retrieval import search

# Load environment variables
load_dotenv()

# Configure logging
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL),
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# =============================================================================
# Budget Constants (T023)
# =============================================================================
# Context Window Budget (8,192 total for Llama 3.2 3B)
# Calculation: 400 + 4000 + 2500 + 1000 = 7900 tokens
# Safety margin: 292 tokens (3.6%) for tokenizer variance

BUDGET_SYSTEM = 400      # Fixed system prompt
BUDGET_CONTEXT = 4000    # Retrieved chunks (highest priority)
BUDGET_HISTORY = 2500    # Conversation history (sliding window)
BUDGET_RESPONSE = 1000   # Buffer for current exchange
BUDGET_TOTAL = 7900      # Conservative limit (8192 - 292 margin)


# =============================================================================
# Tokenizer (T020)
# =============================================================================
_encoder = None


def get_encoder():
    """Get or initialize the tiktoken encoder."""
    global _encoder
    if _encoder is None:
        _encoder = tiktoken.get_encoding("cl100k_base")
    return _encoder


def count_tokens(text: str) -> int:
    """Count tokens in text using tiktoken cl100k_base encoding.

    Note: This is an approximation for Llama models.
    Research showed <10% variance from actual tokenizer.

    Args:
        text: The text to count tokens for

    Returns:
        Number of tokens
    """
    if not text:
        return 0
    return len(get_encoder().encode(text))


# =============================================================================
# OpenRouter Client (T019)
# =============================================================================
def create_openrouter_client() -> AsyncOpenAI:
    """Create an AsyncOpenAI client configured for OpenRouter.

    Returns:
        AsyncOpenAI client with OpenRouter base URL

    Raises:
        ConfigurationError: If OPENROUTER_API_KEY is not set
    """
    api_key = os.getenv("OPENROUTER_API_KEY")
    if not api_key:
        raise ConfigurationError("OPENROUTER_API_KEY environment variable not set")

    return AsyncOpenAI(
        base_url="https://openrouter.ai/api/v1",
        api_key=api_key
    )


# =============================================================================
# Retrieval Integration (T021, T022)
# =============================================================================
async def retrieve_context(
    query: str,
    top_k: int = 5,
    threshold: float = 0.3
) -> RetrievalContext:
    """Retrieve relevant context from vector database.

    Args:
        query: User's search query
        top_k: Maximum number of results
        threshold: Minimum similarity score

    Returns:
        RetrievalContext with results and token counts

    Raises:
        RetrievalError: If vector search fails
    """
    logger.debug(f"Retrieving context for: {query}")
    try:
        results = search(query, top_k=top_k, threshold=threshold)
        total_tokens = sum(count_tokens(r.text) for r in results)
        logger.debug(f"Retrieved {len(results)} results, {total_tokens} tokens")
        return RetrievalContext(
            query=query,
            results=results,
            total_tokens=total_tokens
        )
    except Exception as e:
        raise RetrievalError(f"Vector search failed: {e}")


def format_context_for_prompt(context: RetrievalContext) -> str:
    """Format retrieved context for LLM prompt.

    Format:
    Context from the book:

    [1] Source: {title} ({url})
    {text}

    [2] Source: {title} ({url})
    {text}

    Args:
        context: Retrieved context with results

    Returns:
        Formatted context string
    """
    if not context.results:
        return "No relevant context found in the book."

    lines = ["Context from the book:\n"]
    for i, r in enumerate(context.results, 1):
        lines.append(f"[{i}] Source: {r.title} ({r.url})")
        lines.append(r.text)
        lines.append("")

    return "\n".join(lines)


# =============================================================================
# Response Generation (T024)
# =============================================================================
@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=10),
    retry=retry_if_exception_type((RateLimitError, APIConnectionError))
)
async def generate_response(messages: List[dict], config: AgentConfig) -> str:
    """Generate response from LLM with retry logic.

    Retries up to 3 times with exponential backoff (1s, 2s, 4s).

    Args:
        messages: Messages in OpenAI format
        config: Agent configuration

    Returns:
        Generated response text

    Raises:
        GenerationError: On non-recoverable failure
    """
    try:
        client = create_openrouter_client()
        logger.debug(f"Generating response with model {config.model}")
        response = await client.chat.completions.create(
            model=config.model,
            messages=messages,
            max_tokens=config.max_tokens,
            temperature=config.temperature
        )
        return response.choices[0].message.content
    except (RateLimitError, APIConnectionError):
        logger.warning("Rate limit or connection error, retrying...")
        raise  # Let tenacity retry
    except Exception as e:
        raise GenerationError(f"LLM generation failed: {e}")


# =============================================================================
# Citation Extraction (T025)
# =============================================================================
def extract_citations(response: str, context: RetrievalContext) -> List[Citation]:
    """Extract citations from response text.

    Parses pattern: [Source: {title}]({url})
    Validates against available sources in context.

    Args:
        response: LLM response text
        context: Retrieved context with available sources

    Returns:
        List of validated Citation objects
    """
    pattern = r'\[Source:\s*([^\]]+)\]\(([^)]+)\)'
    matches = re.findall(pattern, response)

    # Build lookup from context
    available_urls = {r.url for r in context.results}

    citations = []
    for title, url in matches:
        if url in available_urls:
            # Find matching result for score
            result = next((r for r in context.results if r.url == url), None)
            citations.append(Citation(
                title=title.strip(),
                url=url,
                score=result.score if result else 0.0
            ))

    logger.debug(f"Extracted {len(citations)} citations from response")
    return citations


# =============================================================================
# Message Building (T026)
# =============================================================================
def build_messages_array(
    state: AgentState,
    query: str,
    context: RetrievalContext
) -> List[dict]:
    """Build messages array for LLM call with budget management.

    Args:
        state: Current agent state
        query: Current user query
        context: Retrieved context

    Returns:
        Messages in OpenAI format
    """
    messages = []

    # 1. System prompt
    system_content = state.config.system_prompt
    messages.append({"role": "system", "content": system_content})

    # 2. Add retrieved context as a system message
    context_content = format_context_for_prompt(context)
    context_tokens = count_tokens(context_content)

    # Truncate context if over budget
    if context_tokens > BUDGET_CONTEXT:
        # Keep only top results that fit
        truncated_results = []
        current_tokens = 0
        for r in context.results:
            r_tokens = count_tokens(r.text) + 50  # overhead for formatting
            if current_tokens + r_tokens <= BUDGET_CONTEXT:
                truncated_results.append(r)
                current_tokens += r_tokens
        truncated_context = RetrievalContext(
            query=context.query,
            results=truncated_results,
            total_tokens=current_tokens
        )
        context_content = format_context_for_prompt(truncated_context)

    messages.append({"role": "system", "content": context_content})

    # 3. Add conversation history (trimmed to budget)
    state.conversation.trim_to_budget(BUDGET_HISTORY)
    for msg in state.conversation.messages:
        if msg.role != "system":  # Skip system messages from history
            messages.append(msg.to_api_format())

    # 4. Add current query
    messages.append({"role": "user", "content": query})

    return messages


# =============================================================================
# Query Processing (T027)
# =============================================================================
async def process_query(
    state: AgentState,
    query: str
) -> tuple[str, List[Citation]]:
    """Process user query through RAG pipeline.

    Pipeline:
    1. Count tokens in query
    2. Retrieve context from vector DB
    3. Build messages array with budget management
    4. Generate LLM response
    5. Extract citations
    6. Update conversation state

    Args:
        state: Current agent state
        query: User's question

    Returns:
        Tuple of (response_text, citations)

    Raises:
        RetrievalError: If vector search fails
        GenerationError: If LLM call fails
    """
    logger.info(f"Processing query: {query[:50]}...")

    # 1. Count query tokens
    query_tokens = count_tokens(query)
    logger.debug(f"Query tokens: {query_tokens}")

    # 2. Retrieve context
    context = await retrieve_context(
        query,
        top_k=state.config.retrieval_top_k,
        threshold=state.config.retrieval_threshold
    )
    state.last_retrieval = context

    # Check for low-relevance results (out-of-scope handling)
    if context.results and all(r.score < 0.3 for r in context.results):
        logger.debug("Low relevance results detected")

    # 3. Build messages
    messages = build_messages_array(state, query, context)

    # 4. Generate response
    response = await generate_response(messages, state.config)

    # 5. Extract citations
    citations = extract_citations(response, context)

    # 6. Update state
    user_msg = Message(
        role="user",
        content=query,
        token_count=query_tokens
    )
    assistant_msg = Message(
        role="assistant",
        content=response,
        citations=citations,
        token_count=count_tokens(response)
    )
    state.conversation.add_message(user_msg)
    state.conversation.add_message(assistant_msg)
    state.total_tokens_used += query_tokens + count_tokens(response)

    logger.info(f"Generated response with {len(citations)} citations")
    return response, citations


# =============================================================================
# Agent Creation (T028)
# =============================================================================
def create_agent(config: AgentConfig = None) -> AgentState:
    """Create and initialize a new agent instance.

    Args:
        config: Optional agent configuration (uses defaults if not provided)

    Returns:
        Initialized agent state

    Raises:
        ConfigurationError: If required environment variables are missing
    """
    if config is None:
        config = AgentConfig()

    # Validate environment
    if not os.getenv("OPENROUTER_API_KEY"):
        raise ConfigurationError("OPENROUTER_API_KEY environment variable not set")

    logger.info(f"Creating agent with model: {config.model}")

    return AgentState(
        config=config,
        conversation=Conversation()
    )


# =============================================================================
# Error Handling (T029)
# =============================================================================
def handle_error(error: Exception) -> ErrorResponse:
    """Convert exception to user-friendly error response.

    Args:
        error: Caught exception

    Returns:
        ErrorResponse with appropriate user message
    """
    if isinstance(error, ConfigurationError):
        return ErrorResponse(
            error_type="config",
            message=str(error),
            user_message="Configuration error. Please check your environment variables.",
            recoverable=False
        )
    elif isinstance(error, RetrievalError):
        return ErrorResponse(
            error_type="retrieval",
            message=str(error),
            user_message="Could not search the book content. Please try again.",
            recoverable=True
        )
    elif isinstance(error, GenerationError):
        return ErrorResponse(
            error_type="generation",
            message=str(error),
            user_message="Could not generate a response. Please try again.",
            recoverable=True
        )
    else:
        return ErrorResponse(
            error_type="unknown",
            message=str(error),
            user_message="An unexpected error occurred. Please try again.",
            recoverable=True
        )


# =============================================================================
# CLI Interface (T039-T044)
# =============================================================================
def print_help():
    """Display available commands."""
    print("""
Available commands:
  quit, exit, bye, q  - Exit the assistant
  clear, reset        - Clear conversation history
  help, ?             - Show this help message

Ask any question about the Physical AI & Humanoid Robotics!
    """)


async def run_cli_async() -> int:
    """Async implementation of CLI loop.

    Returns:
        Exit code (0 for success, 1 for error)
    """
    try:
        agent = create_agent()
    except ConfigurationError as e:
        print(f"Configuration error: {e}")
        print("Please ensure OPENROUTER_API_KEY is set in your .env file.")
        return 1

    print("Book Assistant (type 'quit' to exit, 'clear' to reset, 'help' for commands)")
    print()

    while True:
        try:
            query = input("> ").strip()

            if not query:
                continue

            if query.lower() in ["quit", "exit", "bye", "q"]:
                print("Goodbye!")
                break

            if query.lower() in ["clear", "reset"]:
                agent.conversation = Conversation()
                print("Conversation cleared.")
                continue

            if query.lower() in ["help", "?"]:
                print_help()
                continue

            # Process query
            print("Thinking...")
            response, citations = await process_query(agent, query)
            print()
            print(response)
            print()

        except KeyboardInterrupt:
            print("\nGoodbye!")
            break
        except AgentError as e:
            error = handle_error(e)
            print(f"\n{error.user_message}\n")
            logger.error(f"Agent error: {error.message}")
        except Exception as e:
            print(f"\nAn unexpected error occurred: {e}\n")
            logger.exception("Unexpected error")

    return 0


def run_cli() -> int:
    """Synchronous entry point for CLI.

    Returns:
        Exit code (0 for success, 1 for error)
    """
    return asyncio.run(run_cli_async())


if __name__ == "__main__":
    import sys
    sys.exit(run_cli())
