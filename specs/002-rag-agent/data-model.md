# Data Model: OpenAI Agent with RAG Retrieval Integration

**Feature Branch**: `002-rag-agent`
**Date**: 2025-12-28
**Status**: Complete

## Entity Relationship Overview

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   AgentConfig   │────▶│   AgentState    │────▶│  Conversation   │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                                        │
                                                        ▼
                                                ┌─────────────────┐
                                                │    Message      │
                                                └─────────────────┘
                                                        │
                                                        ▼
                                                ┌─────────────────┐
                                                │    Citation     │
                                                └─────────────────┘

┌─────────────────┐     ┌─────────────────┐
│ RetrievalResult │◀────│ RetrievalContext│
└─────────────────┘     └─────────────────┘

┌─────────────────┐
│  ErrorResponse  │
└─────────────────┘
```

## Entity Definitions

### AgentConfig

Configuration for the conversational agent. Immutable after initialization.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| model | str | Yes | "meta-llama/llama-3.2-3b-instruct:free" | OpenRouter model identifier |
| base_url | str | Yes | "https://api.groq.com/openai/v1" | OpenRouter API endpoint |
| temperature | float | No | 0.7 | Response creativity (0.0-1.0) |
| max_tokens | int | No | 2048 | Maximum tokens per response |
| system_prompt | str | Yes | See below | Agent behavior instructions |
| context_window | int | No | 8192 | Total context window size |
| retrieval_top_k | int | No | 5 | Number of chunks to retrieve |
| retrieval_threshold | float | No | 0.3 | Minimum similarity score |

**Default System Prompt**:
```
You are a helpful assistant for the Isaac Sim robotics book. Your role is to answer questions based ONLY on the provided context from the book.

Rules:
1. Only answer questions using information from the provided context
2. Always cite your sources using the format: [Source: title](url)
3. If the context doesn't contain relevant information, say "I don't have information about that in the book content"
4. Do not answer questions outside the scope of robotics and Isaac Sim
5. Be concise but thorough in your answers
```

**Validation Rules**:
- temperature must be between 0.0 and 1.0
- max_tokens must be positive and less than context_window
- retrieval_top_k must be between 1 and 20
- retrieval_threshold must be between 0.0 and 1.0

---

### AgentState

Runtime state of the agent. Mutable during conversation.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| config | AgentConfig | Yes | - | Agent configuration |
| conversation | Conversation | Yes | - | Current conversation |
| total_tokens_used | int | No | 0 | Cumulative tokens used |
| session_start | datetime | No | now() | Session start timestamp |
| last_retrieval | RetrievalContext | No | None | Last retrieval results |

**State Transitions**:
- `IDLE` → `RETRIEVING` (on user query)
- `RETRIEVING` → `GENERATING` (on retrieval complete)
- `GENERATING` → `IDLE` (on response complete)
- `*` → `ERROR` (on any failure)
- `ERROR` → `IDLE` (on recovery/retry)

---

### Conversation

Represents an ongoing dialogue session.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| id | str | Yes | uuid4() | Unique conversation identifier |
| messages | List[Message] | Yes | [] | Ordered message history |
| started_at | datetime | Yes | now() | Conversation start time |
| total_tokens | int | No | 0 | Total tokens in conversation |

**Methods**:
- `add_message(message: Message)` → Adds message and updates token count
- `get_messages_for_api()` → Returns messages in OpenAI format
- `trim_to_budget(budget: int)` → Removes oldest messages until under budget
- `clear()` → Removes all messages except system prompt

**Invariants**:
- Messages are ordered by timestamp (oldest first)
- First message (if any) should be system message
- total_tokens reflects sum of all message token counts

---

### Message

A single turn in the conversation.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| id | str | Yes | uuid4() | Unique message identifier |
| role | str | Yes | - | Message role: "system", "user", "assistant" |
| content | str | Yes | - | Message text content |
| timestamp | datetime | Yes | now() | Message creation time |
| citations | List[Citation] | No | [] | Citations in this message |
| token_count | int | No | 0 | Token count for this message |
| metadata | dict | No | {} | Additional metadata |

**Validation Rules**:
- role must be one of: "system", "user", "assistant"
- content cannot be empty for user/assistant messages
- token_count should match actual token count of content

**Methods**:
- `to_api_format()` → Returns `{"role": role, "content": content}`

---

### Citation

A source reference within a message.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| title | str | Yes | - | Page/document title |
| url | str | Yes | - | Source URL |
| score | float | No | 0.0 | Retrieval similarity score |
| excerpt | str | No | "" | Relevant text excerpt |
| chunk_id | str | No | "" | Reference to source chunk |

**Format String**: `[Source: {title}]({url})`

**Validation Rules**:
- url must be a valid URL format
- score must be between 0.0 and 1.0
- excerpt should be <= 200 characters

---

### RetrievalContext

Context retrieved for a query.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| query | str | Yes | - | Original user query |
| results | List[RetrievalResult] | Yes | [] | Retrieved chunks |
| total_tokens | int | No | 0 | Total tokens in context |
| retrieved_at | datetime | Yes | now() | Retrieval timestamp |

**Methods**:
- `format_for_prompt()` → Returns formatted context string for LLM
- `trim_to_budget(budget: int)` → Removes lowest-scoring results until under budget
- `get_citations()` → Extracts Citation objects from results

**Format for Prompt**:
```
Context from book:

[1] Source: {title} ({url})
{text}

[2] Source: {title} ({url})
{text}

...
```

---

### RetrievalResult (from Spec-1)

A single retrieved chunk. Imported from `models.py`.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| chunk_id | str | Yes | Unique chunk identifier |
| text | str | Yes | Chunk content |
| score | float | Yes | Similarity score |
| url | str | Yes | Source page URL |
| title | str | Yes | Source page title |
| chunk_index | int | Yes | Chunk position in page |

---

### ErrorResponse

Standardized error structure for user-facing errors.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| error_type | str | Yes | - | Error category |
| message | str | Yes | - | Technical error message |
| user_message | str | Yes | - | User-friendly message |
| retry_after | int | No | None | Seconds until retry (rate limits) |
| recoverable | bool | No | True | Whether retry may succeed |

**Error Types**:
| Type | User Message | Recoverable |
|------|--------------|-------------|
| `rate_limit` | "The service is busy. Please wait {retry_after} seconds and try again." | Yes |
| `api_error` | "There was a problem connecting to the AI service. Please try again." | Yes |
| `retrieval_error` | "Could not search the book content. Please try again." | Yes |
| `auth_error` | "Authentication failed. Please check your API keys." | No |
| `invalid_input` | "Your question couldn't be processed. Please rephrase." | No |
| `context_overflow` | "Your conversation is too long. Type 'clear' to start fresh." | No |

---

## Token Budget Allocation

| Component | Budget | Description |
|-----------|--------|-------------|
| System Prompt | 500 | Fixed agent instructions |
| Retrieved Context | 4000 | Top-k chunks from retrieval |
| Conversation History | 2500 | Previous messages (sliding window) |
| Current Exchange | 1200 | User query + assistant response buffer |
| **Total** | **8192** | Context window limit |

## Data Flow

```
User Query
    │
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 1. Token Count Query                                            │
│    - Count tokens in user query                                 │
│    - Check budget availability                                  │
└─────────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. Retrieve Context (retrieval.py)                              │
│    - search(query, top_k=5, threshold=0.3)                      │
│    - Returns List[RetrievalResult]                              │
│    - Create RetrievalContext with results                       │
└─────────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. Build Messages Array                                         │
│    - System prompt (fixed)                                      │
│    - Retrieved context (formatted)                              │
│    - Conversation history (trimmed to budget)                   │
│    - Current user query                                         │
└─────────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. Generate Response (OpenRouter)                               │
│    - Send messages to LLM                                       │
│    - Receive response                                           │
│    - Extract citations from response                            │
└─────────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. Update State                                                 │
│    - Add user message to conversation                           │
│    - Add assistant message with citations                       │
│    - Update total_tokens_used                                   │
└─────────────────────────────────────────────────────────────────┘
    │
    ▼
Response to User
```

## Python Dataclass Definitions

```python
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional, Dict
import uuid

@dataclass
class Citation:
    title: str
    url: str
    score: float = 0.0
    excerpt: str = ""
    chunk_id: str = ""

    def __str__(self) -> str:
        return f"[Source: {self.title}]({self.url})"

@dataclass
class Message:
    role: str  # "system" | "user" | "assistant"
    content: str
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    timestamp: datetime = field(default_factory=datetime.now)
    citations: List[Citation] = field(default_factory=list)
    token_count: int = 0
    metadata: Dict = field(default_factory=dict)

    def to_api_format(self) -> dict:
        return {"role": self.role, "content": self.content}

@dataclass
class Conversation:
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    messages: List[Message] = field(default_factory=list)
    started_at: datetime = field(default_factory=datetime.now)
    total_tokens: int = 0

@dataclass
class RetrievalContext:
    query: str
    results: List  # List[RetrievalResult] from Spec-1
    total_tokens: int = 0
    retrieved_at: datetime = field(default_factory=datetime.now)

@dataclass
class AgentConfig:
    model: str = "meta-llama/llama-3.2-3b-instruct:free"
    base_url: str = "https://api.groq.com/openai/v1"
    temperature: float = 0.7
    max_tokens: int = 2048
    system_prompt: str = ""
    context_window: int = 8192
    retrieval_top_k: int = 5
    retrieval_threshold: float = 0.3

@dataclass
class AgentState:
    config: AgentConfig
    conversation: Conversation
    total_tokens_used: int = 0
    session_start: datetime = field(default_factory=datetime.now)
    last_retrieval: Optional[RetrievalContext] = None

@dataclass
class ErrorResponse:
    error_type: str
    message: str
    user_message: str
    retry_after: Optional[int] = None
    recoverable: bool = True
```
