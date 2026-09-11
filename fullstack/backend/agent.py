"""RAG Agent for conversational Q&A over book content.

This module implements a conversational AI agent that:
- Retrieves relevant content from a Qdrant vector database
- Generates grounded responses using Groq LLMs
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
# Context Window Budget (8,192 total for Groq models)
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

    Note: This is an approximation for Groq models (Llama, Mixtral, Gemma).
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
# Groq Client (T019)
# =============================================================================
def create_groq_client() -> AsyncOpenAI:
    """Create an AsyncOpenAI client configured for Groq.

    Returns:
        AsyncOpenAI client with Groq base URL

    Raises:
        ConfigurationError: If GROQ_API_KEY is not set
    """
    api_key = os.getenv("GROQ_API_KEY")
    if not api_key:
        raise ConfigurationError("GROQ_API_KEY environment variable not set")

    return AsyncOpenAI(
        base_url="https://api.groq.com/openai/v1",
        api_key=api_key
    )


# =============================================================================
# Book Structure Constants & Query Helpers
# =============================================================================
BOOK_MODULES: List[RetrievalResult] = [
    RetrievalResult(
        chunk_id="mod-intro",
        text="Introduction: Foundations of Physical AI covers embodied intelligence, physical AI fundamentals, and the humanoid robotics autonomy pipeline.",
        score=1.0,
        url="/docs/intro",
        title="Introduction: Foundations of Physical AI",
        chunk_index=0
    ),
    RetrievalResult(
        chunk_id="mod-1",
        text="Module 1: The Robotic Nervous System covers ROS 2 fundamentals, middleware architecture, computational graphs (nodes, topics, services, actions), and AI agents integration.",
        score=1.0,
        url="/docs/module-1/index",
        title="Module 1: The Robotic Nervous System",
        chunk_index=0
    ),
    RetrievalResult(
        chunk_id="mod-2",
        text="Module 2: Robot Kinematics & Physical Structure covers robot links, joints, coordinate frames, forward and inverse kinematics, joint constraints, and URDF modeling for real and simulated robots.",
        score=1.0,
        url="/docs/module-2/index",
        title="Module 2: Robot Kinematics & Physical Structure",
        chunk_index=0
    ),
    RetrievalResult(
        chunk_id="mod-3",
        text="Module 3: The Digital Twin covers Gazebo simulation setup, physics & collision modeling, navigation and motion planning, and Unity visualization.",
        score=1.0,
        url="/docs/module-3/index",
        title="Module 3: The Digital Twin",
        chunk_index=0
    ),
    RetrievalResult(
        chunk_id="mod-4",
        text="Module 4: Perception Systems for Robots covers robot camera models (RGB, Depth, Stereo), LiDAR fundamentals, IMU data and sensor fusion, and building perception pipelines in ROS 2.",
        score=1.0,
        url="/docs/module-4/index",
        title="Module 4: Perception Systems for Robots",
        chunk_index=0
    ),
    RetrievalResult(
        chunk_id="mod-5",
        text="Module 5: The AI-Robot Brain (NVIDIA Isaac) covers NVIDIA Isaac Sim overview and architecture, synthetic data generation, Isaac ROS hardware-accelerated VSLAM, and Nav2 path planning.",
        score=1.0,
        url="/docs/module-5/index",
        title="Module 5: The AI-Robot Brain (NVIDIA Isaac)",
        chunk_index=0
    ),
    RetrievalResult(
        chunk_id="mod-6",
        text="Module 6: Vision–Language–Action (VLA) covers VLA fundamentals, voice-to-action systems, cognitive planning, and executing language plans in ROS 2.",
        score=1.0,
        url="/docs/module-6/index",
        title="Module 6: Vision–Language–Action (VLA)",
        chunk_index=0
    ),
]


def is_overview_query(query: str) -> bool:
    """Check if query is asking for book overview, contents, or table of contents."""
    q = query.lower()
    patterns = [
        "content of the book", "contents of the book", "content of book",
        "table of content", "table of contents",
        "list the content", "list content", "list the chapters", "list chapters",
        "list the modules", "list modules", "list all modules",
        "what does this book cover", "what is this book about", "book overview",
        "what modules", "what topics are covered", "outline of the book",
        "summary of the book", "index of the book", "show me the modules",
        "chapters of the book", "modules of the book"
    ]
    return any(p in q for p in patterns)


def is_greeting_or_identity_query(query: str) -> bool:
    """Check if query is a simple greeting or identity question."""
    q = query.strip().lower()
    greetings = {"hi", "hello", "hey", "hola", "salam", "hi there", "hello there", "good morning", "good evening", "good afternoon"}
    if q in greetings:
        return True
    identity_patterns = ["who are you", "what are you", "what is your name", "what can you do", "how can you help"]
    return any(p in q for p in identity_patterns)


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

    # Fast path: simple greetings or pure identity questions do not require vector search
    if is_greeting_or_identity_query(query) and not is_overview_query(query):
        return RetrievalContext(
            query=query,
            results=[],
            total_tokens=0
        )

    try:
        results = search(query, top_k=top_k, threshold=threshold)

        # If vector search yielded no results for an overview query, provide book modules
        if not results and is_overview_query(query):
            results = BOOK_MODULES

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

    Args:
        context: Retrieved context with results

    Returns:
        Formatted context string
    """
    if not context.results:
        return (
            "No specific excerpt retrieved from vector search. "
            "If the user is greeting you, asking about your identity, or asking for the book overview or table of contents, "
            "answer helpfully using your knowledge of the book structure without saying you lack information."
        )

    lines = ["Context from the book:\n"]
    for i, r in enumerate(context.results, 1):
        lines.append(f"Source: {r.title}")
        lines.append(f"URL: {r.url}")
        lines.append(f"Content:\n{r.text}")
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
        client = create_groq_client()
        logger.debug(f"Generating response with Groq model {config.model}")
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

    Parses pattern: [Source: {title}]({url}) or [{title}]({url})
    Validates against available sources in context or core book modules.

    Args:
        response: LLM response text
        context: Retrieved context with available sources

    Returns:
        List of validated Citation objects
    """
    if not response:
        return []

    # Include context results as well as core book modules so overview answers have clickable links
    candidates = list(context.results) if context.results else []
    for m in BOOK_MODULES:
        if not any(c.url == m.url for c in candidates):
            candidates.append(m)

    available_results_by_url = {r.url.strip(): r for r in candidates}
    citations = []
    seen_urls = set()

    # Pattern: [Source: title](url) or [title](url)
    pattern = r'\[(?:Source:\s*)?([^\]]+)\]\(([^)]+)\)'
    for title_match, raw_url in re.findall(pattern, response):
        # Clean any inner brackets or numbers e.g. [1](... or [2]
        url = re.sub(r'^\[\d+\]\(?', '', raw_url).strip().rstrip(')')
        clean_title = title_match.strip()

        matched_result = None
        for cand_url, r in available_results_by_url.items():
            if url == cand_url or cand_url.endswith(url) or url.endswith(cand_url):
                matched_result = r
                break

        if matched_result and matched_result.url not in seen_urls:
            citations.append(Citation(
                title=clean_title if clean_title else matched_result.title,
                url=matched_result.url,
                score=matched_result.score
            ))
            seen_urls.add(matched_result.url)

    # Fallback: check if context URLs appear directly in the response text
    if not citations:
        for r in candidates:
            if r.url and r.url in response and r.url not in seen_urls:
                citations.append(Citation(
                    title=r.title,
                    url=r.url,
                    score=r.score
                ))
                seen_urls.add(r.url)

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
    if not os.getenv("GROQ_API_KEY"):
        raise ConfigurationError("GROQ_API_KEY environment variable not set")

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
        print("Please ensure GROQ_API_KEY is set in your .env file.")
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
