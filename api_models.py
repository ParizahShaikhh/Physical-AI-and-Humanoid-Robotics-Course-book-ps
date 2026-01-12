"""
Pydantic models for API requests and responses
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict
from datetime import datetime


class SourceReference(BaseModel):
    """Represents a reference to a source document"""
    title: str = Field(..., description="Title of the source")
    url: str = Field(..., description="URL of the source", format="uri")
    preview: Optional[str] = Field(None, description="Brief preview of the content")
    score: float = Field(..., ge=0.0, le=1.0, description="Relevance score")


class QueryRequest(BaseModel):
    """Represents a query request from the frontend to the RAG agent"""
    query: str = Field(..., min_length=1, description="The user's question or query text")
    mode: str = Field("full-book", description="Query mode, either 'full-book' or 'selected-text'",
                      pattern="^(full-book|selected-text)$")
    filters: Optional[Dict] = Field({}, description="Additional filters for selected-text mode")
    top_k: int = Field(5, ge=1, le=20, description="Number of results to retrieve")
    context: Optional[str] = Field("", description="Additional context from the current page")


class SelectedTextRequest(BaseModel):
    """Specialized request for selected-text queries"""
    query: str = Field(..., min_length=1, description="The user's question about the selected text")
    selection_context: str = Field(..., min_length=1, description="The selected text that provides context")
    source_url: Optional[str] = Field(None, description="URL of the source page if applicable")
    filters: Optional[Dict] = Field({}, description="Additional filters")


class QueryResponse(BaseModel):
    """Represents the response from the RAG agent"""
    answer: str = Field(..., description="The answer text from the RAG agent")
    sources: List[SourceReference] = Field(..., description="List of sources used in the response")
    confidence_level: str = Field(..., description="Confidence level",
                                 pattern="^(Very Low|Low|Medium|High)$")
    confidence_score: float = Field(..., ge=0.0, le=1.0, description="Numerical confidence score")
    query_id: Optional[str] = Field(None, description="Unique identifier for the query")
    timestamp: str = Field(..., description="ISO 8601 timestamp of response generation")
    processing_time_ms: Optional[float] = Field(None, description="Time taken to process the query in milliseconds")


class ErrorResponse(BaseModel):
    """Represents an error response"""
    error_code: str = Field(..., description="Standard error code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[dict] = Field(None, description="Additional error details")
    timestamp: str = Field(..., description="ISO 8601 timestamp")