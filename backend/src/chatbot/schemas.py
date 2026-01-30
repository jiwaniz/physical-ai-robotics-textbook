"""
Pydantic schemas for RAG chatbot endpoints.
"""

from datetime import datetime
from typing import List, Optional

from pydantic import BaseModel, Field


# ==================== REQUEST SCHEMAS ====================


class AskRequest(BaseModel):
    """Request to ask a question."""

    query: str = Field(..., min_length=1, max_length=2000, description="User's question")
    selected_text: Optional[str] = Field(
        None,
        max_length=5000,
        description="Text highlighted by user for context",
    )
    page_path: Optional[str] = Field(
        None,
        description="Current page path for additional context",
    )
    conversation_id: Optional[str] = Field(
        None,
        description="Conversation ID for chat history",
    )


class GenerateQuizRequest(BaseModel):
    """Request to generate a quiz from page content."""

    page_content: str = Field(..., min_length=100, description="Page content to generate quiz from")
    page_path: str = Field(..., description="Page path for reference")
    num_questions: int = Field(default=5, ge=3, le=10)


class QuizAnswerRequest(BaseModel):
    """Request to submit quiz answers."""

    quiz_id: str
    answers: List[str] = Field(..., min_length=1)


# ==================== RESPONSE SCHEMAS ====================


class SourceDocument(BaseModel):
    """A source document used to answer the question."""

    content: str
    page_path: str
    score: float


class AskResponse(BaseModel):
    """Response to a question."""

    answer: str
    sources: List[SourceDocument]
    conversation_id: str


class QuizQuestion(BaseModel):
    """A single quiz question."""

    id: int
    question: str
    options: List[str]
    correct_index: int


class GenerateQuizResponse(BaseModel):
    """Response with generated quiz."""

    quiz_id: str
    questions: List[QuizQuestion]
    page_path: str


class QuizResultResponse(BaseModel):
    """Response with quiz results."""

    quiz_id: str
    score: int
    total: int
    percentage: float
    feedback: List[str]


# ==================== DATABASE MODELS ====================


class ChatMessage(BaseModel):
    """Chat message for history."""

    role: str  # 'user' or 'assistant'
    content: str
    timestamp: datetime

    class Config:
        from_attributes = True


class ConversationHistory(BaseModel):
    """Conversation history."""

    conversation_id: str
    messages: List[ChatMessage]
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
