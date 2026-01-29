"""
SQLAlchemy base models and shared database models.
"""

from datetime import datetime
from enum import Enum as PyEnum
from typing import Any, List, Optional

from sqlalchemy import JSON, Boolean, Column, DateTime
from sqlalchemy import Enum as SQLEnum
from sqlalchemy import Float, ForeignKey, Integer, String, Text
from sqlalchemy.ext.declarative import as_declarative, declared_attr
from sqlalchemy.orm import Mapped, mapped_column, relationship


@as_declarative()
class Base:
    """Base class for all SQLAlchemy models."""

    id: Any
    __name__: str

    # Generate __tablename__ automatically from class name
    @declared_attr
    def __tablename__(cls) -> str:
        return cls.__name__.lower()


class TimestampMixin:
    """Mixin to add created_at and updated_at timestamps."""

    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(
        DateTime,
        default=datetime.utcnow,
        onupdate=datetime.utcnow,
        nullable=False,
    )


class BaseModel(Base, TimestampMixin):
    """
    Base model with ID and timestamps.
    All application models should inherit from this.
    """

    __abstract__ = True

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)

    def to_dict(self) -> dict:
        """Convert model to dictionary."""
        return {column.name: getattr(self, column.name) for column in self.__table__.columns}


class User(BaseModel):
    """User model for authentication."""

    __tablename__ = "users"

    email: Mapped[str] = mapped_column(String(255), unique=True, nullable=False, index=True)
    password_hash: Mapped[str] = mapped_column(String(255), nullable=False)
    name: Mapped[str] = mapped_column(String(255), nullable=False)
    is_active: Mapped[bool] = mapped_column(Boolean, default=True, nullable=False)

    # Email verification
    email_verified: Mapped[bool] = mapped_column(Boolean, default=False, nullable=False)
    verification_token: Mapped[Optional[str]] = mapped_column(String(255), nullable=True)
    verification_token_expires: Mapped[Optional[datetime]] = mapped_column(DateTime, nullable=True)

    # Note: UserProfile uses supabase_user_id (UUID from Supabase), not a FK to this table.
    # The relationship is removed since profiles are linked via Supabase user IDs, not local user IDs.


class UserProfile(BaseModel):
    """User profile model for storing background and preferences."""

    __tablename__ = "user_profiles"

    # Supabase user ID (UUID string) - no FK since users are in Supabase
    supabase_user_id: Mapped[str] = mapped_column(
        String(36),
        unique=True,
        nullable=False,
        index=True,
    )
    software_level: Mapped[Optional[str]] = mapped_column(String(20))
    hardware_level: Mapped[Optional[str]] = mapped_column(String(20))
    topics: Mapped[Optional[dict]] = mapped_column(JSON, default=list)


# ==================== QUIZ ENUMS ====================


class QuestionType(str, PyEnum):
    """Types of quiz questions."""

    MULTIPLE_CHOICE = "multiple_choice"
    SHORT_ANSWER = "short_answer"
    CODE_COMPLETION = "code_completion"


class QuestionCategory(str, PyEnum):
    """Content categories for questions (40/30/30 split per spec)."""

    CONCEPTUAL = "conceptual"
    CODE_COMPREHENSION = "code_comprehension"
    TROUBLESHOOTING = "troubleshooting"


class ScoringStatus(str, PyEnum):
    """Scoring status for answers."""

    PENDING = "pending"
    AUTO_GRADED = "auto_graded"
    MANUALLY_GRADED = "manually_graded"


# ==================== QUIZ MODELS ====================


class Quiz(BaseModel):
    """Weekly quiz associated with a specific week of content."""

    __tablename__ = "quizzes"

    # Content association
    week_number: Mapped[int] = mapped_column(Integer, nullable=False, index=True)
    chapter: Mapped[str] = mapped_column(String(100), nullable=False)

    # Metadata
    title: Mapped[str] = mapped_column(String(255), nullable=False)
    description: Mapped[Optional[str]] = mapped_column(Text)

    # Configuration
    time_limit_minutes: Mapped[int] = mapped_column(Integer, default=20)
    max_attempts: Mapped[int] = mapped_column(Integer, default=2)
    passing_score: Mapped[float] = mapped_column(Float, default=60.0)
    is_published: Mapped[bool] = mapped_column(Boolean, default=False)

    # Availability window
    available_from: Mapped[Optional[datetime]] = mapped_column(DateTime)
    available_until: Mapped[Optional[datetime]] = mapped_column(DateTime)

    # Relationships
    questions: Mapped[List["Question"]] = relationship(
        "Question",
        back_populates="quiz",
        cascade="all, delete-orphan",
        order_by="Question.order_index",
    )
    attempts: Mapped[List["QuizAttempt"]] = relationship(
        "QuizAttempt",
        back_populates="quiz",
        cascade="all, delete-orphan",
    )


class Question(BaseModel):
    """Individual quiz question with flexible content structure."""

    __tablename__ = "questions"

    quiz_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("quizzes.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )

    # Question metadata
    question_type: Mapped[QuestionType] = mapped_column(
        SQLEnum(QuestionType, values_callable=lambda e: [x.value for x in e]),
        nullable=False,
    )
    category: Mapped[QuestionCategory] = mapped_column(
        SQLEnum(QuestionCategory, values_callable=lambda e: [x.value for x in e]),
        nullable=False,
    )

    # Question content (JSON structure varies by type)
    content: Mapped[dict] = mapped_column(JSON, nullable=False)

    # Scoring
    points: Mapped[float] = mapped_column(Float, default=1.0)

    # Ordering
    order_index: Mapped[int] = mapped_column(Integer, default=0)

    # Relationships
    quiz: Mapped["Quiz"] = relationship("Quiz", back_populates="questions")
    answers: Mapped[List["Answer"]] = relationship(
        "Answer",
        back_populates="question",
        cascade="all, delete-orphan",
    )


class QuizAttempt(BaseModel):
    """User's attempt at a quiz. Tracks timing and overall score."""

    __tablename__ = "quiz_attempts"

    quiz_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("quizzes.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )
    # Supabase user ID (UUID string) - no FK since users are in Supabase
    user_id: Mapped[str] = mapped_column(
        String(36),
        nullable=False,
        index=True,
    )

    # Attempt tracking
    attempt_number: Mapped[int] = mapped_column(Integer, nullable=False)
    started_at: Mapped[datetime] = mapped_column(DateTime, nullable=False)
    submitted_at: Mapped[Optional[datetime]] = mapped_column(DateTime)

    # Scoring
    score: Mapped[Optional[float]] = mapped_column(Float)
    max_score: Mapped[Optional[float]] = mapped_column(Float)
    percentage: Mapped[Optional[float]] = mapped_column(Float)

    # Status
    is_submitted: Mapped[bool] = mapped_column(Boolean, default=False)
    is_fully_graded: Mapped[bool] = mapped_column(Boolean, default=False)

    # Relationships
    quiz: Mapped["Quiz"] = relationship("Quiz", back_populates="attempts")
    answers: Mapped[List["Answer"]] = relationship(
        "Answer",
        back_populates="attempt",
        cascade="all, delete-orphan",
    )


class Answer(BaseModel):
    """User's answer to a specific question within an attempt."""

    __tablename__ = "answers"

    attempt_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("quiz_attempts.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )
    question_id: Mapped[int] = mapped_column(
        Integer,
        ForeignKey("questions.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )

    # Answer content (JSON for flexibility)
    response: Mapped[dict] = mapped_column(JSON, nullable=False)

    # Scoring
    points_earned: Mapped[Optional[float]] = mapped_column(Float)
    scoring_status: Mapped[ScoringStatus] = mapped_column(
        SQLEnum(ScoringStatus, values_callable=lambda e: [x.value for x in e]),
        default=ScoringStatus.PENDING,
    )

    # Feedback
    feedback: Mapped[Optional[str]] = mapped_column(Text)
    graded_by: Mapped[Optional[int]] = mapped_column(Integer)
    graded_at: Mapped[Optional[datetime]] = mapped_column(DateTime)

    # Relationships
    attempt: Mapped["QuizAttempt"] = relationship("QuizAttempt", back_populates="answers")
    question: Mapped["Question"] = relationship("Question", back_populates="answers")


# Additional models will be added in later phases:
# - Phase 5 (US3): Conversation, Message, Feedback
