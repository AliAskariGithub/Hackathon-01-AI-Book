# Implementation Plan: OpenAI Agent with RAG Retrieval Integration

**Branch**: `002-rag-agent` | **Date**: 2025-12-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/2-rag-agent/spec.md`

## Summary

Build a conversational AI agent that answers questions about the Isaac Sim robotics book by retrieving relevant content from the Qdrant vector database (Spec-1) and generating grounded responses using a free-tier LLM via OpenRouter. The agent uses the OpenAI Agents SDK with base_url override to route requests through OpenRouter while maintaining OpenAI SDK compatibility.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: OpenAI Agents SDK (0.0.x), openai (1.0+), tiktoken OR transformers, tenacity
**Storage**: In-memory conversation state (no persistence)
**Testing**: pytest with async support, pytest-cov for coverage
**Target Platform**: CLI application (cross-platform)
**Project Type**: Single project (extends fullstack/backend)
**Performance Goals**: Response time <15 seconds (free-tier model)
**Constraints**: 8,192 token context window, rate limits on free tier
**Scale/Scope**: Single-user CLI, ~100 queries/session max

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 0 completes.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Spec-First Development | ✅ PASS | spec.md complete with 21 FRs, 6 SCs |
| II. AI-Assisted Development | ✅ PASS | Using Claude Code for implementation |
| III. Technical Accuracy | ⏳ PENDING | Requires Phase 0 research validation |
| IV. Reproducible Workflows | ✅ PASS | quickstart.md documents setup |
| V. Clean Architecture | ✅ PASS | No hardcoded secrets, .env config |
| VI. Modular Code | ✅ PASS | Separate retrieval, agent, models |

**Gate Result**: ⏳ CONDITIONAL PASS - Principle III requires Phase 0 research completion

## Project Structure

### Documentation (this feature)

```text
specs/2-rag-agent/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technology decisions (updated in Phase 0)
├── data-model.md        # Entity definitions
├── quickstart.md        # Setup guide
├── contracts/
│   └── agent-api.md     # Function signatures
└── checklists/
    └── requirements.md  # Validation checklist
```

### Source Code (repository root)

```text
fullstack/backend/
├── agent.py              # Main agent implementation (NEW)
├── retrieval.py          # Vector search (Spec-1)
├── models.py             # Data models (extended with exceptions)
├── main.py               # Ingestion pipeline (Spec-1)
├── .env                  # Environment variables
├── .env.example          # Template (updated)
├── pyproject.toml        # Dependencies (updated)
└── test/
    ├── test_main.py      # Ingestion tests (Spec-1)
    ├── test_retrieval.py # Retrieval tests (Spec-1)
    └── test_agent.py     # Agent tests (NEW)
```

**Structure Decision**: Extends existing `fullstack/backend/` from Spec-1. Single file (`agent.py`) contains all agent logic with models and exceptions in `models.py`.

## Complexity Tracking

No violations requiring justification. Implementation follows minimal approach:
- Single file for agent logic
- Reuses Spec-1 retrieval module
- No external frameworks beyond OpenAI SDK

---

## Pre-Implementation Gates

### Gate 0: Spec-1 Validation

**Purpose**: Ensure Spec-1 dependencies are functional before starting implementation.

**Validation Command**:
```bash
cd fullstack/backend && python -c "from retrieval import search; r=search('What is URDF?', top_k=1); print(f'Found {len(r)} results: {r[0].title if r else \"None\"}')"
```

**Expected Output**: `Found 1 results: URDF Mapping - Part 1` (or similar)

**Gate Criteria**:
- [ ] `retrieval.py` imports without error
- [ ] `search()` function returns `List[RetrievalResult]`
- [ ] Results contain `text`, `url`, `title`, `score` fields
- [ ] Qdrant collection has indexed data

**Action if Failed**: Complete Spec-1 implementation before proceeding.

---

## Phase 0: Research Tasks

**Purpose**: Validate technical assumptions before implementation begins.

**GATE**: Phase 0 must complete successfully before Step 1.

### Research Task 0.1: OpenAI Agents SDK Compatibility

**Question**: Does OpenAI Agents SDK support `set_default_openai_client()` for OpenRouter routing?

**Investigation**:
```bash
pip install openai-agents
python -c "from agents import set_default_openai_client, set_default_openai_api; print('SDK imports work')"
```

**Expected**: Both functions importable and callable.

**Fallback Plan**: If SDK incompatible with OpenRouter:
- Implement custom agent loop using `openai.AsyncOpenAI` directly
- Skip SDK agent patterns, use direct chat completions API
- Update Step 3 to use fallback implementation

### Research Task 0.2: OpenRouter API Test

**Question**: Does OpenRouter respond correctly with `meta-llama/llama-3.2-3b-instruct:free`?

**Investigation**:
```python
import os
from openai import OpenAI

client = OpenAI(
    base_url="https://openrouter.ai/api/v1",
    api_key=os.getenv("OPENROUTER_API_KEY")
)

response = client.chat.completions.create(
    model="meta-llama/llama-3.2-3b-instruct:free",
    messages=[{"role": "user", "content": "Say 'Hello' in one word"}],
    max_tokens=10
)
print(response.choices[0].message.content)
```

**Expected**: Response containing "Hello" or similar.

**Document**: Response latency, rate limit headers, actual model used.

### Research Task 0.3: Tokenizer Selection

**Question**: What tokenizer provides accurate counts for Llama 3.2 models?

**Investigation Options**:

**Option A: tiktoken (cl100k_base)**
```python
import tiktoken
encoder = tiktoken.get_encoding("cl100k_base")
sample = "What is URDF and how is it used in robotics?"
print(f"tiktoken: {len(encoder.encode(sample))} tokens")
```

**Option B: transformers AutoTokenizer**
```python
from transformers import AutoTokenizer
tokenizer = AutoTokenizer.from_pretrained("meta-llama/Llama-3.2-3B-Instruct")
sample = "What is URDF and how is it used in robotics?"
print(f"transformers: {len(tokenizer.encode(sample))} tokens")
```

**Decision Criteria**:
- If counts differ by <10%: Use tiktoken (lighter dependency)
- If counts differ by >10%: Use transformers (more accurate)
- Document: Chosen tokenizer, accuracy comparison, dependency impact

### Research Task 0.4: Validate Retrieval Response Format

**Question**: Does `retrieval.search()` return data in expected format?

**Investigation**:
```python
from retrieval import search
from models import RetrievalResult

results = search("What is URDF?", top_k=3)
for r in results:
    print(f"Type: {type(r)}")
    print(f"Fields: chunk_id={r.chunk_id}, text={r.text[:50]}..., score={r.score}, url={r.url}, title={r.title}")
```

**Expected**: List of `RetrievalResult` objects with all fields populated.

### Research Task 0.5: Update research.md

**Action**: Document all Phase 0 findings in `specs/2-rag-agent/research.md`:
- SDK compatibility result
- OpenRouter test result (latency, success)
- Tokenizer decision with rationale
- Retrieval format confirmation

### Phase 0 Gate Checklist

- [ ] OpenAI Agents SDK compatible with OpenRouter OR fallback plan documented
- [ ] OpenRouter API test successful with target model
- [ ] Tokenizer selected and documented
- [ ] Retrieval format validated
- [ ] research.md updated with findings
- [ ] Constitution Check Principle III re-evaluated

**Action**: Update Constitution Check status after Phase 0 completes.

---

## Implementation Steps

### Step 1: Environment Setup

**Goal**: Install dependencies and configure OpenRouter access

**Prerequisites**: Phase 0 complete, all gates passed.

**Tasks**:
1. Add new dependencies to `pyproject.toml`:
   ```toml
   [project.dependencies]
   openai-agents = ">=0.0.1"  # Or remove if Phase 0 showed incompatibility
   openai = ">=1.0.0"
   tiktoken = ">=0.5.0"       # Or transformers per Phase 0 finding
   tenacity = ">=8.0.0"
   pytest-cov = ">=4.0.0"     # For coverage measurement
   ```

2. Update `.env.example` with:
   ```bash
   # OpenRouter Configuration
   OPENROUTER_API_KEY=sk-or-v1-your-key-here

   # Existing from Spec-1
   COHERE_API_KEY=your-cohere-key
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your-qdrant-key
   ```

3. Create verification script to test OpenRouter connectivity

**Acceptance Criteria**:
- [ ] `pip install -e .` succeeds
- [ ] OpenRouter test call returns response
- [ ] All environment variables documented
- [ ] Tokenizer dependency matches Phase 0 decision

**Files Modified**: `pyproject.toml`, `.env.example`

---

### Step 2: Agent Configuration Module

**Goal**: Define agent configuration, exceptions, and initialization

**Tasks**:
1. Add custom exceptions to `models.py`:
   ```python
   # Custom Exceptions
   class AgentError(Exception):
       """Base exception for agent errors."""
       pass

   class RetrievalError(AgentError):
       """Error during vector search."""
       pass

   class GenerationError(AgentError):
       """Error during LLM response generation."""
       pass

   class ContextOverflowError(AgentError):
       """Context exceeds token budget."""
       pass

   class ConfigurationError(AgentError):
       """Invalid agent configuration."""
       pass
   ```

2. Add dataclasses to `models.py`:
   - `AgentConfig` (with fallback models list)
   - `AgentState`
   - `Citation`
   - `Message`
   - `Conversation`
   - `RetrievalContext`
   - `ErrorResponse`

3. Define explicit system prompt in `AgentConfig`:
   ```python
   DEFAULT_SYSTEM_PROMPT = """You are an AI assistant for the Isaac Sim Robotics book. Your role is to help users find information from the book content.

   IMPORTANT RULES:
   1. Answer questions using ONLY the provided context from the book
   2. Always cite your sources using this exact format: [Source: Title](URL)
   3. If the context doesn't contain relevant information, say: "I don't have information about that in the book content."
   4. Do not answer questions outside the scope of robotics, Isaac Sim, and related topics covered in the book
   5. Be concise but thorough in your answers
   6. If multiple sources are relevant, cite all of them

   Example citation format:
   "URDF is a format for describing robots [Source: URDF Basics](https://example.com/urdf)."
   """
   ```

4. Implement configuration validation

**Acceptance Criteria**:
- [ ] All exception classes defined in `models.py`
- [ ] All dataclasses defined with type hints
- [ ] System prompt includes citation format instruction
- [ ] System prompt defines behavior boundaries
- [ ] Default values set per spec
- [ ] Validation raises `ConfigurationError` for invalid values
- [ ] Fallback models list configured

**Files Modified**: `models.py`

---

### Step 3: OpenRouter Client Setup

**Goal**: Configure OpenAI SDK to use OpenRouter as backend

**Note**: Implementation pattern depends on Phase 0 research findings. Two approaches documented below.

**Approach A: Using OpenAI Agents SDK** (if Phase 0.1 confirmed compatibility)
```python
from openai import AsyncOpenAI
from agents import set_default_openai_client, set_default_openai_api

client = AsyncOpenAI(
    base_url="https://openrouter.ai/api/v1",
    api_key=os.getenv("OPENROUTER_API_KEY")
)
set_default_openai_client(client)
set_default_openai_api("chat_completions")
```

**Approach B: Direct OpenAI Client** (fallback if SDK incompatible)
```python
from openai import AsyncOpenAI

def create_openrouter_client() -> AsyncOpenAI:
    return AsyncOpenAI(
        base_url="https://openrouter.ai/api/v1",
        api_key=os.getenv("OPENROUTER_API_KEY")
    )
```

**Tasks**:
1. Create `agent.py` with OpenRouter client initialization (per Phase 0 findings)

2. Implement `create_agent()` function with fallback model support:
   ```python
   def create_agent(config: AgentConfig = None) -> AgentState:
       if config is None:
           config = AgentConfig()

       # Validate environment
       if not os.getenv("OPENROUTER_API_KEY"):
           raise ConfigurationError("OPENROUTER_API_KEY not set")

       # Test primary model, fall back if needed
       model = try_model(config.models)  # Tries models in order
       config.model = model

       return AgentState(
           config=config,
           conversation=Conversation()
       )
   ```

3. Implement `try_model()` function for fallback:
   ```python
   def try_model(models: List[str]) -> str:
       """Try models in order, return first that responds."""
       client = create_openrouter_client()
       for model in models:
           try:
               response = client.chat.completions.create(
                   model=model,
                   messages=[{"role": "user", "content": "test"}],
                   max_tokens=5
               )
               return model
           except Exception as e:
               logger.warning(f"Model {model} unavailable: {e}")
               continue
       raise ConfigurationError(f"No available models from: {models}")
   ```

4. Add environment variable validation

**Acceptance Criteria**:
- [ ] Client initializes without error
- [ ] Missing API key raises `ConfigurationError`
- [ ] `create_agent()` returns valid `AgentState`
- [ ] Model fallback works when primary unavailable
- [ ] Implementation matches Phase 0 research findings

**Files Created**: `agent.py`

---

### Step 4: Token Counting

**Goal**: Implement accurate token counting for budget management

**Note**: Tokenizer choice depends on Phase 0.3 research findings.

**Tasks**:
1. Initialize tokenizer based on Phase 0 decision:

   **If tiktoken selected**:
   ```python
   import tiktoken

   _encoder = None

   def get_encoder():
       global _encoder
       if _encoder is None:
           _encoder = tiktoken.get_encoding("cl100k_base")
       return _encoder

   def count_tokens(text: str) -> int:
       """Count tokens using tiktoken cl100k_base encoding.

       Note: This is an approximation for Llama models.
       Phase 0 research showed <10% variance from actual tokenizer.
       """
       if not text:
           return 0
       return len(get_encoder().encode(text))
   ```

   **If transformers selected**:
   ```python
   from transformers import AutoTokenizer

   _tokenizer = None

   def get_tokenizer():
       global _tokenizer
       if _tokenizer is None:
           _tokenizer = AutoTokenizer.from_pretrained("meta-llama/Llama-3.2-3B-Instruct")
       return _tokenizer

   def count_tokens(text: str) -> int:
       """Count tokens using Llama 3.2 tokenizer."""
       if not text:
           return 0
       return len(get_tokenizer().encode(text))
   ```

2. Implement `count_tokens(text: str) -> int`

3. Add token counting to Message creation

**Acceptance Criteria**:
- [ ] Token count matches expected values (within 5% for tiktoken, exact for transformers)
- [ ] Empty string returns 0
- [ ] Unicode text handled correctly
- [ ] Tokenizer matches Phase 0 decision

**Files Modified**: `agent.py`

---

### Step 5: Retrieval Integration

**Goal**: Connect to Spec-1 retrieval module

**Tasks**:
1. Import `search()` from `retrieval.py`

2. Implement `retrieve_context()`:
   ```python
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
       try:
           results = search(query, top_k=top_k, threshold=threshold)
           total_tokens = sum(count_tokens(r.text) for r in results)
           return RetrievalContext(
               query=query,
               results=results,
               total_tokens=total_tokens
           )
       except Exception as e:
           raise RetrievalError(f"Vector search failed: {e}")
   ```

3. Implement `format_context_for_prompt()`:
   ```python
   def format_context_for_prompt(context: RetrievalContext) -> str:
       """Format retrieved context for LLM prompt.

       Format:
       Context from the book:

       [1] Source: {title} ({url})
       {text}

       [2] Source: {title} ({url})
       {text}
       """
       if not context.results:
           return "No relevant context found in the book."

       lines = ["Context from the book:\n"]
       for i, r in enumerate(context.results, 1):
           lines.append(f"[{i}] Source: {r.title} ({r.url})")
           lines.append(r.text)
           lines.append("")

       return "\n".join(lines)
   ```

4. Handle empty retrieval results

**Acceptance Criteria**:
- [ ] `retrieve_context()` returns `RetrievalContext`
- [ ] Formatted context includes source URLs
- [ ] Empty results handled gracefully
- [ ] Token counts calculated for budget management

**Files Modified**: `agent.py`

---

### Step 6: Context Window Management

**Goal**: Implement budget allocation and truncation

**Tasks**:
1. Define budget constants with calculation:
   ```python
   # Context Window Budget (8,192 total for Llama 3.2 3B)
   # Calculation: 400 + 4000 + 2500 + 1000 = 7900 tokens
   # Safety margin: 292 tokens (3.6%) for tokenizer variance

   BUDGET_SYSTEM = 400      # Fixed system prompt
   BUDGET_CONTEXT = 4000    # Retrieved chunks (highest priority)
   BUDGET_HISTORY = 2500    # Conversation history (sliding window)
   BUDGET_RESPONSE = 1000   # Buffer for current exchange
   BUDGET_TOTAL = 7900      # Conservative limit (8192 - 292 margin)

   # Rationale:
   # - System prompt is fixed and essential (~400 tokens)
   # - Retrieved context is core RAG functionality (prioritized)
   # - History enables multi-turn but can be trimmed
   # - Response buffer prevents overflow mid-generation
   ```

2. Implement `build_messages_array()`:
   - Add system prompt
   - Add formatted context (truncate if over budget)
   - Add conversation history (sliding window)
   - Add current query

3. Implement `Conversation.trim_to_budget()`:
   ```python
   def trim_to_budget(self, budget: int) -> None:
       """Remove oldest messages until under token budget.

       Preserves system message if present.
       """
       while self.total_tokens > budget and len(self.messages) > 1:
           # Keep system message (index 0) if present
           if self.messages[0].role == "system":
               if len(self.messages) > 1:
                   removed = self.messages.pop(1)
                   self.total_tokens -= removed.token_count
           else:
               removed = self.messages.pop(0)
               self.total_tokens -= removed.token_count
   ```

**Acceptance Criteria**:
- [ ] Total tokens never exceed 7900 (conservative limit)
- [ ] Budget calculation documented with rationale
- [ ] Highest-scoring chunks kept when truncating context
- [ ] Oldest messages removed first from history
- [ ] System message preserved during trimming

**Files Modified**: `agent.py`, `models.py`

---

### Step 7: Response Generation

**Goal**: Generate LLM responses with retry logic

**Tasks**:
1. Implement `generate_response()` with tenacity:
   ```python
   from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type
   from openai import RateLimitError, APIConnectionError

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
           response = await client.chat.completions.create(
               model=config.model,
               messages=messages,
               max_tokens=config.max_tokens,
               temperature=config.temperature
           )
           return response.choices[0].message.content
       except (RateLimitError, APIConnectionError):
           raise  # Let tenacity retry
       except Exception as e:
           raise GenerationError(f"LLM generation failed: {e}")
   ```

2. Call OpenRouter via AsyncOpenAI client

3. Extract and return response text

**Acceptance Criteria**:
- [ ] Successful response returned
- [ ] Retries on rate limit (up to 3 times)
- [ ] `GenerationError` raised on non-recoverable failure
- [ ] Exceptions imported from models.py

**Files Modified**: `agent.py`

---

### Step 8: Citation Extraction

**Goal**: Extract citations from LLM responses

**Tasks**:
1. Implement `extract_citations()`:
   ```python
   import re

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

       return citations
   ```

2. Add citation validation

**Acceptance Criteria**:
- [ ] Citations extracted from response
- [ ] Invalid/mismatched citations filtered
- [ ] Returns empty list if no citations found

**Files Modified**: `agent.py`

---

### Step 9: Query Processing Pipeline

**Goal**: Orchestrate full RAG pipeline

**Tasks**:
1. Implement `process_query()`:
   ```python
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
           ContextOverflowError: If context exceeds budget
       """
       # 1. Count query tokens
       query_tokens = count_tokens(query)

       # 2. Retrieve context
       context = await retrieve_context(
           query,
           top_k=state.config.retrieval_top_k,
           threshold=state.config.retrieval_threshold
       )
       state.last_retrieval = context

       # 3. Build messages
       messages = build_messages_array(state, query, context)

       # 4. Generate response
       response = await generate_response(messages, state.config)

       # 5. Extract citations
       citations = extract_citations(response, context)

       # 6. Update state
       state.conversation.add_message(Message(role="user", content=query))
       state.conversation.add_message(Message(
           role="assistant",
           content=response,
           citations=citations
       ))
       state.total_tokens_used += count_tokens(query) + count_tokens(response)

       return response, citations
   ```

2. Handle errors and convert to `ErrorResponse`

**Acceptance Criteria**:
- [ ] Returns response and citations
- [ ] State updated with new messages
- [ ] Errors converted to user-friendly messages
- [ ] Token usage tracked

**Files Modified**: `agent.py`

---

### Step 10: CLI Interface

**Goal**: Implement interactive REPL with proper async handling

**Tasks**:
1. Implement async CLI wrapper:
   ```python
   import asyncio
   import sys

   async def run_cli_async() -> int:
       """Async implementation of CLI loop."""
       try:
           agent = create_agent()
       except ConfigurationError as e:
           print(f"Configuration error: {e}")
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

       return 0

   def run_cli() -> int:
       """Synchronous entry point for CLI."""
       return asyncio.run(run_cli_async())

   if __name__ == "__main__":
       sys.exit(run_cli())
   ```

2. Handle commands: quit, clear, help

3. Handle Ctrl+C gracefully

4. Format response output with citations

5. Add help command:
   ```python
   def print_help():
       print("""
   Available commands:
     quit, exit, bye, q  - Exit the assistant
     clear, reset        - Clear conversation history
     help, ?             - Show this help message

   Ask any question about the Isaac Sim robotics book!
       """)
   ```

**Acceptance Criteria**:
- [ ] REPL loop runs until exit command
- [ ] `clear` resets conversation
- [ ] Ctrl+C exits gracefully
- [ ] Responses formatted with citations
- [ ] Main entry point allows `python agent.py` execution
- [ ] Async properly wrapped with asyncio.run()

**Files Modified**: `agent.py`

---

### Step 11: Error Handling

**Goal**: Implement comprehensive error handling using exceptions from Step 2

**Tasks**:
1. Import custom exceptions from `models.py`:
   ```python
   from models import (
       AgentError, RetrievalError, GenerationError,
       ContextOverflowError, ConfigurationError, ErrorResponse
   )
   ```

2. Implement `handle_error()`:
   ```python
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
       elif isinstance(error, ContextOverflowError):
           return ErrorResponse(
               error_type="context_overflow",
               message=str(error),
               user_message="Your conversation is too long. Type 'clear' to start fresh.",
               recoverable=False
           )
       else:
           return ErrorResponse(
               error_type="unknown",
               message=str(error),
               user_message="An unexpected error occurred. Please try again.",
               recoverable=True
           )
   ```

3. Add error handling to all public functions (already done in Steps 3, 5, 7, 9)

**Acceptance Criteria**:
- [ ] Exceptions imported from models.py (not redefined)
- [ ] All errors produce user-friendly messages
- [ ] Technical details logged, not displayed
- [ ] Recoverable errors allow retry

**Files Modified**: `agent.py`

---

### Step 12: Testing

**Goal**: Write comprehensive tests with coverage measurement

**Tasks**:
1. Add pytest-cov to dependencies (done in Step 1)

2. Create `test/test_agent.py` with:
   - Unit tests for token counting
   - Unit tests for citation extraction
   - Integration tests for retrieval
   - Integration tests for response generation
   - CLI command tests
   - Conversation reset tests

3. Add conversation reset test:
   ```python
   def test_conversation_resets_on_clear():
       """Verify conversation state doesn't persist across clears."""
       agent = create_agent()

       # Simulate conversation
       agent.conversation.add_message(Message(role="user", content="test"))
       agent.conversation.add_message(Message(role="assistant", content="response"))

       assert len(agent.conversation.messages) == 2

       # Clear conversation
       agent.conversation = Conversation()

       assert len(agent.conversation.messages) == 0
       assert agent.conversation.total_tokens == 0

   def test_no_file_persistence():
       """Verify no unintended file writes during conversation."""
       import tempfile
       import os

       # Create agent in temp directory
       with tempfile.TemporaryDirectory() as tmpdir:
           original_cwd = os.getcwd()
           os.chdir(tmpdir)

           try:
               # Note: This test may need mocking for API calls
               # Just verify no files created in working directory
               files_before = set(os.listdir(tmpdir))

               agent = create_agent()
               agent.conversation.add_message(Message(role="user", content="test"))

               files_after = set(os.listdir(tmpdir))
               assert files_before == files_after, "Unexpected files created"
           finally:
               os.chdir(original_cwd)
   ```

4. Test queries from spec:
   - Book content queries (6)
   - Follow-up queries (3)
   - Out-of-scope queries (4)
   - Edge cases (4)

5. Run tests with coverage:
   ```bash
   python -m pytest test/test_agent.py --cov=agent --cov-report=term-missing -v
   ```

**Acceptance Criteria**:
- [ ] All unit tests pass
- [ ] Integration tests pass with live APIs
- [ ] Test coverage >80% for agent.py
- [ ] Conversation reset test passes
- [ ] No file persistence test passes

**Files Created**: `test/test_agent.py`

---

### Step 13: Documentation

**Goal**: Update documentation

**Tasks**:
1. Add docstrings to all public functions

2. Update `README.md` with agent usage

3. Add CLI help text

**Acceptance Criteria**:
- [ ] All public functions documented
- [ ] README includes agent section
- [ ] `--help` displays usage

**Files Modified**: `agent.py`, `README.md`

---

## Validation Checkpoints

### Before Step 1 (Spec-1 Gate)
```bash
cd fullstack/backend && python -c "from retrieval import search; r=search('What is URDF?', top_k=1); print(f'OK: {r[0].title}')"
```
Expected: `OK: URDF Mapping - Part 1` (or similar title)

### After Phase 0 (Research Gate)
- [ ] research.md updated with all findings
- [ ] Tokenizer decision documented
- [ ] SDK compatibility confirmed or fallback documented
- [ ] Constitution Check re-evaluated

### After Step 3 (OpenRouter Setup)
```bash
cd fullstack/backend && python -c "from agent import create_agent; a=create_agent(); print(f'Model: {a.config.model}')"
```
Expected: `Model: meta-llama/llama-3.2-3b-instruct:free`

### After Step 5 (Retrieval Integration)
```bash
cd fullstack/backend && python -c "import asyncio; from agent import retrieve_context; c=asyncio.run(retrieve_context('What is URDF?')); print(f'Found {len(c.results)} results, {c.total_tokens} tokens')"
```
Expected: `Found 5 results, XXXX tokens`

### After Step 9 (Full Pipeline)
```bash
cd fullstack/backend && python -c "
import asyncio
from agent import create_agent, process_query

async def test():
    agent = create_agent()
    response, citations = await process_query(agent, 'What is URDF?')
    print(f'Response length: {len(response)}')
    print(f'Citations: {len(citations)}')
    return response

asyncio.run(test())
"
```
Expected: Response length > 100, Citations >= 1

### After Step 10 (CLI)
```bash
cd fullstack/backend && python agent.py
```
Expected: Interactive prompt showing `>`

### After Step 12 (Tests with Coverage)
```bash
cd fullstack/backend && python -m pytest test/test_agent.py --cov=agent --cov-report=term-missing -v
```
Expected: All tests pass, coverage >80%

---

## Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| OpenRouter rate limits | Medium | Medium | Retry with backoff (Step 7), user messaging |
| Model unavailable | Low | High | Fallback model list (Step 3) |
| Context overflow | Low | Medium | Conservative budget (Step 6), user warning |
| Slow response times | Medium | Low | Set expectations in UI, show "Thinking..." |
| OpenAI SDK incompatible | Low | Medium | Fallback to direct client (Phase 0, Step 3) |
| Tokenizer inaccuracy | Low | Low | Phase 0 validation, safety margin in budget |

---

## Dependencies

### Required Before Implementation
- Spec-1 complete (Qdrant populated, retrieval.py working) - Validated by Gate 0
- OpenRouter API key obtained
- Python 3.11+ environment
- Phase 0 research complete

### External Services
- OpenRouter API (LLM)
- Qdrant Cloud (Vector DB)
- Cohere API (Embeddings)

---

## Artifacts Generated

| Artifact | Path | Purpose |
|----------|------|---------|
| research.md | specs/2-rag-agent/research.md | Technology decisions (update in Phase 0) |
| data-model.md | specs/2-rag-agent/data-model.md | Entity definitions |
| agent-api.md | specs/2-rag-agent/contracts/agent-api.md | Function contracts |
| quickstart.md | specs/2-rag-agent/quickstart.md | Setup guide |
| plan.md | specs/2-rag-agent/plan.md | This file |

---

## Next Steps

After `/sp.plan` completes:
1. Run `/sp.tasks` to generate task breakdown
2. Execute Phase 0 research tasks
3. Update research.md with findings
4. Run `/sp.implement` to begin implementation
5. Test each step per validation checkpoints
