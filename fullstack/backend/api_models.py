"""Pydantic models for FastAPI chat API.

This module defines the request/response contracts for the /api/chat endpoint.
Models are separate from agent models to maintain clean API boundaries.
"""

from enum import Enum
from typing import List, Optional
from pydantic import BaseModel, Field
from uuid import UUID


class ErrorType(str, Enum):
    """Error types for API responses (matches spec FR-014)."""
    NETWORK = "network"
    BACKEND = "backend"
    TIMEOUT = "timeout"
    RATE_LIMIT = "rate_limit"


class ChatMessage(BaseModel):
    """Single message in conversation history.

    Sent from client to backend for context.
    Note: timestamps and citations are client-side only.
    """
    role: str = Field(..., pattern="^(user|assistant)$", description="Message sender role")
    content: str = Field(..., min_length=1, description="Message text content")


class Citation(BaseModel):
    """Reference to book content.

    URL is transformed to relative Docusaurus route by backend.
    Score is pass-through from RAG agent, not displayed in MVP.
    """
    title: str = Field(..., description="Source page title")
    url: str = Field(..., description="Relative Docusaurus route (e.g., /docs/page)")
    score: Optional[float] = Field(None, description="Relevance score (not displayed in MVP)")


class ChatRequest(BaseModel):
    """Request body for POST /api/chat endpoint.

    Follows spec FR-001 contract.
    """
    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question (1-2000 chars)"
    )
    conversation_history: List[ChatMessage] = Field(
        default_factory=list,
        description="Previous messages for context"
    )
    conversation_id: Optional[str] = Field(
        None,
        description="Client-generated UUID for tracking"
    )


class ChatResponse(BaseModel):
    """Response body from POST /api/chat endpoint.

    Follows spec FR-001 contract.
    """
    answer: Optional[str] = Field(None, description="LLM-generated response")
    citations: List[Citation] = Field(default_factory=list, description="Source references")
    conversation_id: str = Field(..., description="Echo or generated conversation ID")
    error: Optional[str] = Field(None, description="Error type if request failed")


class ErrorResponse(BaseModel):
    """Error response with typed error information.

    Used for detailed error handling per spec FR-014.
    """
    error_type: ErrorType = Field(..., description="Categorized error type")
    message: str = Field(..., description="User-friendly error message")
    details: Optional[str] = Field(None, description="Technical details for debugging")


class HealthResponse(BaseModel):
    """Response body for GET /health endpoint."""
    status: str = Field(..., description="Health status: 'healthy' or 'unhealthy'")
    version: str = Field(..., description="API version")
    dependencies: dict = Field(default_factory=dict, description="Dependency status")