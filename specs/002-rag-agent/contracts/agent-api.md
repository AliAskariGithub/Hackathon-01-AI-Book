# Agent API Contract: OpenAI Agent with RAG Retrieval Integration

**Feature Branch**: `002-rag-agent`
**Date**: 2025-12-28
**Status**: Complete

## Overview

This document defines the internal API contracts for the RAG agent. Since this is a CLI application (not REST API - that's Spec-3), these contracts define the function signatures and interfaces between components.

## Module: agent.py

### Public Functions

#### `create_agent(config: AgentConfig = None) -> AgentState`

Creates and initializes a new agent instance.

**Parameters**:
| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| config | AgentConfig | No | Default config | Agent configuration |

**Returns**: `AgentState` - Initialized agent state

**Raises**:
- `ValueError` - If config validation fails
- `EnvironmentError` - If required env vars missing

**Example**:
```python
from agent import create_agent, AgentConfig

# With defaults
agent = create_agent()

# With custom config
config = AgentConfig(temperature=0.5, retrieval_top_k=3)
agent = create_agent(config)
```

---

#### `async process_query(state: AgentState, query: str) -> tuple[str, List[Citation]]`

Processes a user query through the RAG pipeline.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| state | AgentState | Yes | Current agent state |
| query | str | Yes | User's question |

**Returns**: `tuple[str, List[Citation]]` - Response text and citations

**Raises**:
- `RetrievalError` - If vector search fails
- `GenerationError` - If LLM call fails
- `ContextOverflowError` - If context exceeds budget

**Side Effects**:
- Updates `state.conversation` with new messages
- Updates `state.total_tokens_used`
- Updates `state.last_retrieval`

**Example**:
```python
response, citations = await process_query(agent, "What is URDF?")
print(response)
for c in citations:
    print(f"  - {c}")
```

---

#### `clear_conversation(state: AgentState) -> None`

Clears conversation history while preserving configuration.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| state | AgentState | Yes | Agent state to clear |

**Returns**: None

**Side Effects**:
- Resets `state.conversation.messages` to empty list
- Resets `state.conversation.total_tokens` to 0
- Preserves `state.config` and `state.session_start`

---

#### `run_cli() -> int`

Main entry point for CLI interface.

**Parameters**: None

**Returns**: `int` - Exit code (0 for success, 1 for error)

**Behavior**:
1. Creates agent with default config
2. Displays welcome message
3. Enters REPL loop
4. Handles user input and commands
5. Displays responses with citations
6. Exits on quit command or Ctrl+C

---

### Internal Functions

#### `async retrieve_context(query: str, top_k: int, threshold: float) -> RetrievalContext`

Retrieves relevant context from vector database.

**Parameters**:
| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| query | str | Yes | - | Search query |
| top_k | int | No | 5 | Max results |
| threshold | float | No | 0.3 | Min similarity |

**Returns**: `RetrievalContext` - Retrieved chunks with metadata

**Delegates to**: `retrieval.search()` from Spec-1

---

#### `format_context_for_prompt(context: RetrievalContext) -> str`

Formats retrieved context for LLM prompt.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| context | RetrievalContext | Yes | Retrieved context |

**Returns**: `str` - Formatted context string

**Format**:
```
Context from the book:

[1] Source: {title} ({url})
{text}

[2] Source: {title} ({url})
{text}
```

---

#### `async generate_response(messages: List[dict], config: AgentConfig) -> str`

Calls LLM to generate response.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| messages | List[dict] | Yes | Messages in OpenAI format |
| config | AgentConfig | Yes | Agent configuration |

**Returns**: `str` - Generated response text

**Retries**: 3 attempts with exponential backoff

---

#### `count_tokens(text: str) -> int`

Counts tokens in text using tiktoken.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| text | str | Yes | Text to count |

**Returns**: `int` - Token count

---

#### `build_messages_array(state: AgentState, query: str, context: RetrievalContext) -> List[dict]`

Builds messages array for LLM call with budget management.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| state | AgentState | Yes | Current agent state |
| query | str | Yes | Current user query |
| context | RetrievalContext | Yes | Retrieved context |

**Returns**: `List[dict]` - Messages in OpenAI format

**Budget Allocation**:
1. System prompt (500 tokens)
2. Retrieved context (up to 4000 tokens)
3. Conversation history (up to 2500 tokens)
4. Current query (counted)
5. Response buffer (1200 tokens reserved)

---

#### `extract_citations(response: str, context: RetrievalContext) -> List[Citation]`

Extracts citations from response text.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| response | str | Yes | LLM response text |
| context | RetrievalContext | Yes | Available sources |

**Returns**: `List[Citation]` - Extracted citations

**Pattern Matched**: `[Source: {title}]({url})`

---

#### `handle_error(error: Exception) -> ErrorResponse`

Converts exceptions to user-friendly error responses.

**Parameters**:
| Name | Type | Required | Description |
|------|------|----------|-------------|
| error | Exception | Yes | Caught exception |

**Returns**: `ErrorResponse` - Standardized error

---

## Integration with Spec-1

### Import Contract

```python
from retrieval import search
from models import RetrievalResult
```

### `search()` Function Signature (from retrieval.py)

```python
def search(
    query: str,
    top_k: int = 5,
    threshold: float = 0.0,
    collection_name: str = "book_embeddings"
) -> List[RetrievalResult]
```

**Expected Return**:
```python
@dataclass
class RetrievalResult:
    chunk_id: str
    text: str
    score: float
    url: str
    title: str
    chunk_index: int
```

---

## Error Contracts

### Custom Exceptions

```python
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

### Error Handling Contract

All public functions should:
1. Catch internal exceptions
2. Convert to `ErrorResponse` using `handle_error()`
3. Display `user_message` to user
4. Log `message` for debugging
5. Retry if `recoverable=True` and within retry budget

---

## CLI Commands Contract

| Command | Aliases | Description | Handler |
|---------|---------|-------------|---------|
| quit | exit, bye, q | Exit application | `sys.exit(0)` |
| clear | reset | Clear conversation | `clear_conversation()` |
| help | ? | Show help | Display command list |

---

## Configuration Contract

### Environment Variables

| Variable | Required | Validation | Error if Missing |
|----------|----------|------------|------------------|
| GROQ_API_KEY | Yes | Non-empty string | ConfigurationError |
| COHERE_API_KEY | Yes | Non-empty string | ConfigurationError |
| QDRANT_URL | Yes | Valid URL | ConfigurationError |
| QDRANT_API_KEY | Yes | Non-empty string | ConfigurationError |
| LOG_LEVEL | No | DEBUG\|INFO\|WARNING\|ERROR | Default: INFO |

### Startup Validation

On `create_agent()`:
1. Load environment variables
2. Validate all required vars present
3. Test OpenRouter connectivity (optional health check)
4. Test retrieval.py import
5. Return initialized AgentState or raise ConfigurationError

---

## Response Format Contract

### Successful Response

```
[Response text with inline citations]

For example: URDF (Unified Robot Description Format) is an XML format used to
describe robot models [Source: URDF Basics](https://example.com/urdf). It defines
the robot's links, joints, and physical properties [Source: Robot Modeling](https://example.com/modeling).
```

### Error Response

```
Sorry, I encountered an issue: {user_message}

[If recoverable] Please try again or type 'help' for assistance.
```

### Out-of-Scope Response

```
I can only answer questions about the Isaac Sim robotics book content.
Your question about {topic} is outside my knowledge scope.

Try asking about topics like:
- URDF and robot description
- Forward/inverse kinematics
- Isaac Sim simulation
- ROS integration
```
