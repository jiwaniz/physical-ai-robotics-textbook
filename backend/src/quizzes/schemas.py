"""
Pydantic schemas for quiz endpoints.
"""

from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field, field_validator


class QuestionType(str, Enum):
    """Types of quiz questions."""

    MULTIPLE_CHOICE = "multiple_choice"
    SHORT_ANSWER = "short_answer"
    CODE_COMPLETION = "code_completion"


class QuestionCategory(str, Enum):
    """Content categories for questions."""

    CONCEPTUAL = "conceptual"
    CODE_COMPREHENSION = "code_comprehension"
    TROUBLESHOOTING = "troubleshooting"


class ScoringStatus(str, Enum):
    """Scoring status for answers."""

    PENDING = "pending"
    AUTO_GRADED = "auto_graded"
    MANUALLY_GRADED = "manually_graded"


# ==================== QUESTION CONTENT SCHEMAS ====================


class MultipleChoiceOption(BaseModel):
    """Single option for multiple choice question."""

    id: str = Field(..., description="Option identifier (a, b, c, d)")
    text: str = Field(..., description="Option text")
    is_correct: bool = Field(default=False, description="Whether this is the correct answer")


class MultipleChoiceContent(BaseModel):
    """Content structure for multiple choice questions."""

    question_text: str
    code_snippet: Optional[str] = None
    options: List[MultipleChoiceOption] = Field(..., min_length=2, max_length=6)
    explanation: Optional[str] = None

    @field_validator("options")
    @classmethod
    def validate_single_correct(cls, v: List[MultipleChoiceOption]) -> List[MultipleChoiceOption]:
        """Ensure exactly one option is marked as correct."""
        correct_count = sum(1 for opt in v if opt.is_correct)
        if correct_count != 1:
            raise ValueError("Exactly one option must be marked as correct")
        return v


class ShortAnswerContent(BaseModel):
    """Content structure for short answer questions."""

    question_text: str
    code_snippet: Optional[str] = None
    expected_keywords: List[str] = Field(default_factory=list)
    rubric: Optional[str] = None
    max_length: int = Field(default=500, ge=50, le=2000)
    sample_answer: Optional[str] = None


class CodeCompletionContent(BaseModel):
    """Content structure for code completion questions."""

    question_text: str
    code_template: str = Field(..., description="Code with blanks to fill")
    language: str = Field(default="python")
    expected_solution: str
    hints: List[str] = Field(default_factory=list)


# ==================== REQUEST SCHEMAS ====================


class QuizCreateRequest(BaseModel):
    """Request to create a new quiz."""

    week_number: int = Field(..., ge=1, le=13)
    chapter: str = Field(..., min_length=1, max_length=100)
    title: str = Field(..., min_length=5, max_length=255)
    description: Optional[str] = None
    time_limit_minutes: int = Field(default=20, ge=5, le=60)
    max_attempts: int = Field(default=2, ge=1, le=5)
    passing_score: float = Field(default=60.0, ge=0, le=100)
    available_from: Optional[datetime] = None
    available_until: Optional[datetime] = None


class QuizUpdateRequest(BaseModel):
    """Request to update quiz metadata."""

    title: Optional[str] = Field(None, min_length=5, max_length=255)
    description: Optional[str] = None
    time_limit_minutes: Optional[int] = Field(None, ge=5, le=60)
    passing_score: Optional[float] = Field(None, ge=0, le=100)
    available_from: Optional[datetime] = None
    available_until: Optional[datetime] = None
    is_published: Optional[bool] = None


class QuestionCreateRequest(BaseModel):
    """Request to create a question."""

    question_type: QuestionType
    category: QuestionCategory
    content: Dict[str, Any]
    points: float = Field(default=1.0, ge=0.5, le=10.0)
    order_index: Optional[int] = None


class QuestionUpdateRequest(BaseModel):
    """Request to update a question."""

    content: Optional[Dict[str, Any]] = None
    points: Optional[float] = Field(None, ge=0.5, le=10.0)
    order_index: Optional[int] = None
    category: Optional[QuestionCategory] = None


class AnswerSubmitRequest(BaseModel):
    """Request to save an answer."""

    question_id: int
    response: Dict[str, Any]


class QuizSubmitRequest(BaseModel):
    """Request to submit quiz attempt."""

    answers: List[AnswerSubmitRequest]


class GradeAnswerRequest(BaseModel):
    """Request to grade an answer manually."""

    points_earned: float = Field(..., ge=0)
    feedback: Optional[str] = None


# ==================== RESPONSE SCHEMAS ====================


class QuestionResponseBase(BaseModel):
    """Base question response (without correct answers for students)."""

    id: int
    question_type: QuestionType
    category: QuestionCategory
    points: float
    order_index: int
    content: Dict[str, Any]

    class Config:
        from_attributes = True


class QuestionResponse(QuestionResponseBase):
    """Full question response (admin view)."""

    created_at: datetime
    updated_at: datetime


class QuizResponse(BaseModel):
    """Quiz summary response."""

    id: int
    week_number: int
    chapter: str
    title: str
    description: Optional[str]
    time_limit_minutes: int
    max_attempts: int
    passing_score: float
    is_published: bool
    available_from: Optional[datetime]
    available_until: Optional[datetime]
    question_count: int = 0
    created_at: datetime

    class Config:
        from_attributes = True


class QuizDetailResponse(QuizResponse):
    """Quiz with questions (for taking)."""

    questions: List[QuestionResponseBase]
    user_attempts_count: int = 0
    user_best_score: Optional[float] = None


class QuizStartResponse(BaseModel):
    """Response when starting a quiz attempt."""

    attempt_id: int
    started_at: datetime
    time_limit_minutes: int
    questions: List[QuestionResponseBase]


class AnswerResponse(BaseModel):
    """Answer in attempt detail."""

    id: int
    question_id: int
    response: Dict[str, Any]
    points_earned: Optional[float]
    scoring_status: ScoringStatus
    feedback: Optional[str]

    class Config:
        from_attributes = True


class QuizAttemptResponse(BaseModel):
    """Quiz attempt summary."""

    id: int
    quiz_id: int
    quiz_title: str
    attempt_number: int
    started_at: datetime
    submitted_at: Optional[datetime]
    score: Optional[float]
    max_score: Optional[float]
    percentage: Optional[float]
    is_submitted: bool
    is_fully_graded: bool

    class Config:
        from_attributes = True


class QuizAttemptDetailResponse(QuizAttemptResponse):
    """Full attempt with answers and grading."""

    answers: List[AnswerResponse]
    questions_with_solutions: List[Dict[str, Any]]


class QuizResultResponse(BaseModel):
    """Response after submitting quiz."""

    attempt_id: int
    score: float
    max_score: float
    percentage: float
    is_fully_graded: bool
    passing_score: float
    passed: bool
    auto_graded_count: int
    pending_grading_count: int


class GradingQueueItem(BaseModel):
    """Item in grading queue."""

    answer_id: int
    attempt_id: int
    quiz_title: str
    question_text: str
    question_type: QuestionType
    student_response: Dict[str, Any]
    points_possible: float
    submitted_at: datetime

    class Config:
        from_attributes = True
