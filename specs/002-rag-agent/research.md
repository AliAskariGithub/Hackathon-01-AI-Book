# Research: OpenAI Agent with RAG Retrieval Integration

**Feature Branch**: `002-rag-agent`
**Date**: 2025-12-28
**Status**: Complete

## Research Questions Resolved

### RQ-001: OpenAI Agents SDK + OpenRouter Integration Approach

**Decision**: Use OpenAI Agents SDK with `set_default_openai_client()` and `set_default_openai_api("chat_completions")`

**Rationale**:
- OpenAI Agents SDK provides production-ready agent patterns with minimal abstractions
- `base_url` override enables routing to OpenRouter without code changes
- Setting API to `chat_completions` ensures compatibility with OpenRouter (which may not support Responses API)
- Maintains OpenAI SDK compatibility for future migration if needed

**Alternatives Considered**:
1. **Custom agent implementation without SDK**: Rejected - more boilerplate, less maintainable
2. **LangChain/LlamaIndex**: Rejected - heavier dependencies, overkill for single-agent use case
3. **Direct OpenRouter API calls**: Rejected - lose agent patterns and conversation management

**Implementation Pattern**:
```python
from openai import AsyncOpenAI
from agents import set_default_openai_client, set_default_openai_api

# Configure OpenRouter as backend
client = AsyncOpenAI(
    base_url="https://api.groq.com/openai/v1",
    api_key=os.getenv("GROQ_API_KEY")
)
set_default_openai_client(client)
set_default_openai_api("chat_completions")
```

**Sources**:
- [OpenAI Agents SDK Configuration](https://openai.github.io/openai-agents-python/config/)
- [OpenRouter API Reference](https://openrouter.ai/docs/api/reference/overview)

---

### RQ-002: Free-Tier Model Selection for OpenRouter

**Decision**: Primary model `meta-llama/llama-3.2-3b-instruct:free`

**Rationale**:
- 8,192 token context window (sufficient for RAG + conversation)
- Free tier with reasonable rate limits
- Good instruction-following for Q&A tasks
- Fast inference for interactive CLI use

**Alternatives Considered**:
1. **mistralai/devstral-2512:free**: Good alternative, similar capabilities
2. **xiaomi/mimo-v2-flash:free**: Faster but smaller context window
3. **google/gemma-2-9b-it:free**: Larger model but may have higher latency

**Fallback Strategy**: If primary model unavailable, try fallbacks in order:
1. `mistralai/devstral-2512:free`
2. `xiaomi/mimo-v2-flash:free`

---

### RQ-003: Token Counting Strategy

**Decision**: Use `tiktoken` library with `cl100k_base` encoding for approximate token counting

**Rationale**:
- `tiktoken` is the standard tokenizer for OpenAI-compatible models
- `cl100k_base` encoding works well for Llama models (close approximation)
- Lightweight, fast counting for real-time budget management
- Already commonly used in OpenAI ecosystem

**Alternatives Considered**:
1. **Model-specific tokenizers**: Rejected - adds complexity, Llama tokenizer requires downloading weights
2. **Character-based estimation (4 chars ≈ 1 token)**: Rejected - too imprecise for budget management
3. **No counting, just truncate on error**: Rejected - poor UX with random failures

**Implementation**:
```python
import tiktoken

encoder = tiktoken.get_encoding("cl100k_base")

def count_tokens(text: str) -> int:
    return len(encoder.encode(text))
```

---

### RQ-004: Context Window Budget Allocation

**Decision**: Fixed budget allocation with sliding window for conversation history

**Budget Breakdown** (8,192 tokens total):
| Component | Tokens | Notes |
|-----------|--------|-------|
| System prompt | 500 | Fixed, includes agent instructions |
| Retrieved context | 4,000 | Variable, highest-scoring chunks first |
| Conversation history | 2,500 | Sliding window, most recent 5-10 exchanges |
| User query + response | 1,200 | Buffer for current turn |

**Rationale**:
- Prioritizes retrieved context (core RAG functionality)
- Sliding window prevents unbounded memory growth
- Fixed system prompt ensures consistent behavior
- Buffer prevents context overflow errors

**Truncation Strategy**:
1. If retrieved context exceeds budget: drop lowest-scoring chunks
2. If conversation history exceeds budget: drop oldest messages (keep system prompt)
3. If still over: truncate oldest message content

---

### RQ-005: Citation Format and Extraction

**Decision**: Inline markdown citations with structured format

**Format**: `[Source: {title}]({url})`

**Rationale**:
- Markdown links render well in CLI and future web UI
- Title provides context without requiring URL lookup
- Consistent format enables automated extraction/validation

**Alternatives Considered**:
1. **Footnote-style citations**: Rejected - harder to read in CLI
2. **JSON citations in response**: Rejected - disrupts natural language flow
3. **Separate citations section**: Rejected - breaks answer-citation association

**Implementation**:
- Include citation format requirement in system prompt
- Post-process response to validate citation presence
- Log citation count for SC-002 metric tracking

---

### RQ-006: Conversation State Management

**Decision**: In-memory list of message dictionaries with role/content

**Data Structure**:
```python
@dataclass
class Message:
    role: str  # "user" | "assistant" | "system"
    content: str
    timestamp: datetime
    citations: List[Citation] = field(default_factory=list)
    token_count: int = 0

@dataclass
class Conversation:
    messages: List[Message] = field(default_factory=list)
    total_tokens: int = 0
    started_at: datetime = field(default_factory=datetime.now)
```

**Rationale**:
- Simple structure compatible with OpenAI message format
- Token tracking enables budget management
- Timestamp enables debugging and metrics
- No persistence required (per spec)

---

### RQ-007: Error Handling and Retry Strategy

**Decision**: 3 retries with exponential backoff for transient failures

**Retry Configuration**:
- Max retries: 3
- Initial delay: 1 second
- Backoff multiplier: 2 (1s → 2s → 4s)
- Max delay: 10 seconds

**Retryable Errors**:
- HTTP 429 (Rate limit)
- HTTP 500, 502, 503, 504 (Server errors)
- Connection timeouts
- Network errors

**Non-Retryable Errors**:
- HTTP 400 (Bad request - fix input)
- HTTP 401, 403 (Auth errors - fix credentials)
- HTTP 404 (Not found - fix model name)

**Implementation**:
```python
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=10),
    retry=retry_if_exception_type((RateLimitError, APIConnectionError))
)
async def call_llm(messages: List[dict]) -> str:
    ...
```

---

### RQ-008: CLI Interface Design

**Decision**: Simple REPL loop with clear prompts and exit commands

**Interface Design**:
```
Book Assistant (type 'quit' to exit, 'clear' to reset)
> What is URDF?

[Response with citations]

> quit
Goodbye!
```

**Commands**:
- `quit`, `exit`, `bye`, `q`: Exit the application
- `clear`, `reset`: Clear conversation history
- `help`: Show available commands
- Ctrl+C: Graceful exit

**Rationale**:
- Minimal, familiar interface
- Clear visual separation between user input and responses
- Multiple exit options for convenience

---

## Technology Stack Summary

| Component | Technology | Version |
|-----------|------------|---------|
| Language | Python | 3.11+ |
| Agent Framework | OpenAI Agents SDK | 0.0.x |
| LLM Client | openai | 1.0+ |
| LLM Provider | OpenRouter | - |
| Model | meta-llama/llama-3.2-3b-instruct:free | - |
| Tokenizer | tiktoken | 0.5+ |
| Retry Logic | tenacity | 8.0+ |
| Env Config | python-dotenv | 1.0+ |
| Vector Search | Spec-1 retrieval.py | - |

## Dependencies

### New Dependencies (add to pyproject.toml)
```toml
[project.dependencies]
openai-agents = ">=0.0.1"
openai = ">=1.0.0"
tiktoken = ">=0.5.0"
tenacity = ">=8.0.0"
```

### Existing Dependencies (from Spec-1)
- cohere
- qdrant-client
- python-dotenv

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| GROQ_API_KEY | Yes | OpenRouter API key for LLM access |
| COHERE_API_KEY | Yes | Cohere API key (used by retrieval.py) |
| QDRANT_URL | Yes | Qdrant Cloud cluster URL |
| QDRANT_API_KEY | Yes | Qdrant Cloud API key |
| LOG_LEVEL | No | Logging level (default: INFO) |

---

## Phase 0 Validation Results (2025-12-28)

### T003: OpenAI Agents SDK Compatibility
- **Status**: ✅ VERIFIED
- **Package**: openai-agents v0.6.4
- **Import Pattern**: `from agents import set_default_openai_client, set_default_openai_api`
- **Both functions are available and importable**

### T004: OpenRouter API Test
- **Status**: ⏳ PENDING - Requires GROQ_API_KEY in .env
- **Action**: User must add GROQ_API_KEY to fullstack/backend/.env

### T005: Tokenizer Selection
- **Status**: ✅ VERIFIED
- **Decision**: Use tiktoken with cl100k_base encoding
- **Rationale**:
  - Lightweight (no model weights download required)
  - Fast initialization
  - Reasonable approximation for Llama models (<10% variance expected)
  - transformers tokenizer would require downloading multi-GB model weights
- **Test Results**:
  - "What is URDF and how is it used in robotics?": 12 tokens
  - "The quick brown fox jumps over the lazy dog.": 10 tokens
  - "Hello world!": 3 tokens
  - "": 0 tokens

### T005b: Tokenizer Wrapper Implementation
- **Pattern**:
```python
import tiktoken

_encoder = None

def get_encoder():
    global _encoder
    if _encoder is None:
        _encoder = tiktoken.get_encoding("cl100k_base")
    return _encoder

def count_tokens(text: str) -> int:
    if not text:
        return 0
    return len(get_encoder().encode(text))
```
