# Tasks: OpenAI Agent with RAG Retrieval Integration

**Branch**: `002-rag-agent` | **Date**: 2025-12-28 | **Plan**: [plan.md](./plan.md)

## User Stories Summary

| Story | Priority | Description |
|-------|----------|-------------|
| US1 | P1 | Single Question Answering - Core Q&A with citations |
| US2 | P1 | Multi-turn Conversation - Context retention |
| US3 | P2 | Out-of-Scope Query Handling - Graceful boundaries |

---

## Phase 1: Pre-Implementation Gates

**Goal**: Validate Spec-1 dependencies before starting.

- [x] T001 Validate Spec-1 retrieval works: `cd fullstack/backend && python -c "from retrieval import search; r=search('test', top_k=1); print(f'OK: {len(r)} results')"`
- [x] T002 Verify Qdrant collection has data via retrieval validation: `cd fullstack/backend && python retrieval.py --validate`

---

## Phase 2: Research (Phase 0 from plan.md)

**Goal**: Discover and validate technical assumptions before implementation.

- [x] T003 Discover OpenAI Agents SDK API: Install `pip install openai-agents` and document actual import pattern (may differ from assumption). Record available functions in research.md
- [ ] T004 Test OpenRouter API with target model: Requires GROQ_API_KEY in .env (user action needed)
- [x] T005 Compare tiktoken vs transformers tokenizer accuracy for Llama 3.2 and document choice with rationale
- [x] T005b Implement chosen tokenizer wrapper function based on T005 findings (creates reusable count_tokens pattern for T020)
- [x] T006 Document all findings in `specs/2-rag-agent/research.md` including SDK API, tokenizer choice, and fallback decisions

---

## Phase 3: Setup

**Goal**: Configure environment and dependencies.

- [x] T007 Add dependencies to `fullstack/backend/pyproject.toml`: openai-agents (or fallback per T003), openai>=1.0.0, tiktoken>=0.5.0 (or transformers per T005), tenacity>=8.0.0, pytest-cov>=4.0.0
- [x] T008 Update `fullstack/backend/.env.example` with GROQ_API_KEY=sk-or-v1-your-key-here
- [x] T009 Run `cd fullstack/backend && pip install -e .` and verify installation succeeds

---

## Phase 4: Foundational (Blocking)

**Goal**: Create shared models and exceptions required by all user stories.

- [x] T010 Add exception classes to `fullstack/backend/models.py`: AgentError (base), RetrievalError, GenerationError, ContextOverflowError, ConfigurationError
- [x] T011 [P] Add Citation dataclass to `fullstack/backend/models.py` with fields: title, url, score, excerpt, chunk_id
- [x] T012 [P] Add Message dataclass to `fullstack/backend/models.py` with fields: id, role, content, timestamp, citations, token_count, metadata
- [x] T013 [P] Add Conversation dataclass to `fullstack/backend/models.py` with fields: id, messages, started_at, total_tokens
- [x] T014 [P] Add RetrievalContext dataclass to `fullstack/backend/models.py` with fields: query, results, total_tokens, retrieved_at
- [x] T015 [P] Add AgentConfig dataclass to `fullstack/backend/models.py` with DEFAULT_SYSTEM_PROMPT (see template below), model, base_url, temperature, max_tokens, context_window, retrieval_top_k, retrieval_threshold, models (fallback list)
- [x] T016 [P] Add AgentState dataclass to `fullstack/backend/models.py` with fields: config, conversation, total_tokens_used, session_start, last_retrieval
- [x] T017 [P] Add ErrorResponse dataclass to `fullstack/backend/models.py` with fields: error_type, message, user_message, retry_after, recoverable

**T015 System Prompt Template** (reference plan.md Step 2):
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

---

## Phase 5: User Story 1 - Single Question Answering (P1)

**Goal**: Enable single question Q&A with citations.

**Independent Test**: Ask "What is URDF?" and verify response includes citation.

### Core Implementation

- [x] T018 [US1] Create `fullstack/backend/agent.py` with imports (os, logging, asyncio, re, typing, openai, tenacity) and logging setup
- [x] T019 [US1] Implement `create_openrouter_client() -> AsyncOpenAI` returning client with base_url and api_key in `fullstack/backend/agent.py`
- [x] T020 [US1] Implement `count_tokens(text: str) -> int` using tokenizer from T005b in `fullstack/backend/agent.py` (depends on T005b)
- [x] T021 [US1] Implement `async retrieve_context(query, top_k=5, threshold=0.3) -> RetrievalContext` calling retrieval.search in `fullstack/backend/agent.py`
- [x] T022 [US1] Implement `format_context_for_prompt(context: RetrievalContext) -> str` with numbered sources in `fullstack/backend/agent.py`
- [x] T023 [US1] Define budget constants in `fullstack/backend/agent.py`: BUDGET_SYSTEM=400, BUDGET_CONTEXT=4000, BUDGET_HISTORY=2500, BUDGET_RESPONSE=1000, BUDGET_TOTAL=7900
- [x] T024 [US1] Implement `async generate_response(messages: List[dict], config: AgentConfig) -> str` with @retry decorator (3 attempts, exponential backoff 1-10s) in `fullstack/backend/agent.py` (see plan.md Step 7)
- [x] T025 [US1] Implement `extract_citations(response: str, context: RetrievalContext) -> List[Citation]` parsing `[Source: title](url)` pattern in `fullstack/backend/agent.py`
- [x] T026 [US1] Implement `build_messages_array(state, query, context) -> List[dict]` with budget management in `fullstack/backend/agent.py`
- [x] T027 [US1] Implement `async process_query(state: AgentState, query: str) -> tuple[str, List[Citation]]` orchestrating RAG pipeline in `fullstack/backend/agent.py`
- [x] T028 [US1] Implement `create_agent(config: AgentConfig = None) -> AgentState` with model fallback via try_model() in `fullstack/backend/agent.py`
- [x] T029 [US1] Implement `handle_error(error: Exception) -> ErrorResponse` mapping exceptions to user messages in `fullstack/backend/agent.py`

### Validation

- [ ] T030 [US1] Test single question with full async pattern:
```bash
cd fullstack/backend && python -c "
import asyncio
import os
from agent import create_agent, process_query

async def test():
    agent = create_agent()
    response, citations = await process_query(agent, 'What is URDF?')
    print(f'Response: {response[:200]}...')
    print(f'Citations: {len(citations)}')
    assert len(citations) >= 1, 'Expected at least 1 citation'
    print('✓ US1 validation passed')

asyncio.run(test())
"
```

---

## Phase 6: User Story 2 - Multi-turn Conversation (P1)

**Goal**: Enable context retention across conversation turns.

**Independent Test**: Ask follow-up "Tell me more about that" after initial question.

### Implementation

- [x] T031 [US2] Add `add_message(message: Message) -> None` method to Conversation in `fullstack/backend/models.py` (updates messages list and total_tokens)
- [x] T032 [US2] Add `trim_to_budget(budget: int) -> None` method to Conversation in `fullstack/backend/models.py` (removes oldest non-system messages)
- [x] T033 [US2] Update `process_query()` to store user and assistant messages in conversation in `fullstack/backend/agent.py`
- [x] T034 [US2] Update `build_messages_array()` to include trimmed conversation history before current query in `fullstack/backend/agent.py`

### Validation

- [ ] T035 [US2] Test multi-turn conversation:
```bash
cd fullstack/backend && python -c "
import asyncio
from agent import create_agent, process_query

async def test():
    agent = create_agent()
    r1, _ = await process_query(agent, 'What is URDF?')
    print(f'Q1: {r1[:100]}...')
    r2, _ = await process_query(agent, 'Tell me more about that')
    print(f'Q2: {r2[:100]}...')
    assert 'urdf' in r2.lower() or 'robot' in r2.lower(), 'Expected context retention'
    print('✓ US2 validation passed')

asyncio.run(test())
"
```

---

## Phase 7: User Story 3 - Out-of-Scope Handling (P2)

**Goal**: Gracefully handle off-topic queries.

**Independent Test**: Ask "What's the weather today?" and verify polite decline.

### Implementation

- [x] T036 [US3] Verify system prompt in AgentConfig includes out-of-scope handling rules (rule #3 and #4 in template) in `fullstack/backend/models.py`
- [x] T037 [US3] Add low-relevance detection in `process_query()`: if all results score < 0.3, prepend note to context in `fullstack/backend/agent.py`

### Validation

- [ ] T038 [US3] Test out-of-scope query:
```bash
cd fullstack/backend && python -c "
import asyncio
from agent import create_agent, process_query

async def test():
    agent = create_agent()
    response, citations = await process_query(agent, 'What is the weather today?')
    print(f'Response: {response}')
    keywords = ['don\\'t have', 'cannot', 'outside', 'only answer', 'book content']
    assert any(k in response.lower() for k in keywords), 'Expected graceful decline'
    print('✓ US3 validation passed')

asyncio.run(test())
"
```

---

## Phase 8: CLI Interface

**Goal**: Interactive command-line interface.

- [x] T039 Implement `print_help() -> None` displaying available commands in `fullstack/backend/agent.py`
- [x] T040 Implement `async run_cli_async() -> int` with REPL loop (input prompt "> ", process query, display response) in `fullstack/backend/agent.py`
- [x] T041 Implement `run_cli() -> int` sync wrapper calling `asyncio.run(run_cli_async())` in `fullstack/backend/agent.py`
- [x] T042 Add `if __name__ == "__main__": import sys; sys.exit(run_cli())` entry point in `fullstack/backend/agent.py`
- [x] T043 Handle commands in run_cli_async(): quit/exit/bye/q (break), clear/reset (new Conversation), help/? (print_help) in `fullstack/backend/agent.py`
- [x] T044 Handle KeyboardInterrupt (Ctrl+C) gracefully with "Goodbye!" message in `fullstack/backend/agent.py`

### Validation

- [ ] T045 Run `cd fullstack/backend && python agent.py` and test: type question, type "help", type "clear", type "quit"

---

## Phase 9: Testing

**Goal**: Comprehensive test coverage >80%.

- [x] T046 Create `fullstack/backend/test/test_agent.py` with pytest imports and fixtures for mocking OpenRouter
- [x] T047 Add unit test `test_count_tokens()` verifying empty string=0, known string≈expected in `fullstack/backend/test/test_agent.py`
- [x] T048 Add unit test `test_extract_citations()` with sample response containing `[Source: Title](url)` pattern in `fullstack/backend/test/test_agent.py`
- [x] T049 Add test `test_conversation_resets_on_clear()` verifying messages list empties after reset in `fullstack/backend/test/test_agent.py`
- [x] T050 Add test `test_no_file_persistence()` verifying no files created during conversation in `fullstack/backend/test/test_agent.py`
- [x] T051 Run tests with coverage: `cd fullstack/backend && python -m pytest test/test_agent.py --cov=agent --cov-report=term-missing -v`

---

## Phase 10: Documentation

**Goal**: Complete documentation.

- [x] T052 Add docstrings to all public functions in `fullstack/backend/agent.py` (create_agent, process_query, run_cli, etc.)
- [x] T053 Update `fullstack/backend/README.md` with "RAG Agent" section: usage (`python agent.py`), commands, example session

---

## Dependencies

```
Phase 1 (Gates) ──► Phase 2 (Research) ──► Phase 3 (Setup) ──► Phase 4 (Foundational)
     │                    │                                            │
     │                    │                                            │
     │               T005 ──► T005b ─────────────────────────────────► T020
     │                                                                 │
     └────────────────────────────────────────────────────────────────┴──────────┐
                                                                                  │
                    ┌─────────────────────────────────────────────────────────────┴─────────────────────┐
                    │                                                                                   │
                    ▼                                                                                   ▼
            Phase 5 (US1: Q&A)                                                                Phase 6 (US2: Multi-turn)
                    │                                                                                   │
                    └───────────────────────────┬───────────────────────────────────────────────────────┘
                                                │
                                                ▼
                                    Phase 7 (US3: Out-of-Scope)
                                                │
                                                ▼
                                      Phase 8 (CLI Interface)
                                                │
                                                ▼
                                       Phase 9 (Testing)
                                                │
                                                ▼
                                    Phase 10 (Documentation)
```

**Critical Path**: T005 → T005b → T020 (tokenizer must be decided before count_tokens implementation)

## Parallel Execution Opportunities

### Within Phase 4 (Foundational)
```
T010 ──► T011, T012, T013, T014, T015, T016, T017 (all [P] after exceptions)
```

### Within Phase 5 (US1)
```
T018 ──► T019, T020 [P] (T020 requires T005b complete)
T019, T020 ──► T021, T022, T023 [P]
T021, T022, T023 ──► T024, T025, T026 [P]
T024, T025, T026 ──► T027
T027 ──► T028, T029 [P]
```

### Between Phases 5 & 6
```
US1 (Phase 5) and US2 (Phase 6) can be partially parallelized:
- T031, T032 (conversation methods) can start after T013
- Full US2 validation requires US1 completion
```

---

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 54 |
| Phase 1 (Gates) | 2 |
| Phase 2 (Research) | 5 |
| Phase 3 (Setup) | 3 |
| Phase 4 (Foundational) | 8 |
| Phase 5 (US1) | 13 |
| Phase 6 (US2) | 5 |
| Phase 7 (US3) | 3 |
| Phase 8 (CLI) | 7 |
| Phase 9 (Testing) | 6 |
| Phase 10 (Docs) | 2 |

**MVP Scope**: Phases 1-5 (US1 only) = 31 tasks for basic Q&A functionality

**Files Created**:
- `fullstack/backend/agent.py` (new)
- `fullstack/backend/test/test_agent.py` (new)

**Files Modified**:
- `fullstack/backend/models.py`
- `fullstack/backend/pyproject.toml`
- `fullstack/backend/.env.example`
- `fullstack/backend/README.md`
- `specs/2-rag-agent/research.md`
