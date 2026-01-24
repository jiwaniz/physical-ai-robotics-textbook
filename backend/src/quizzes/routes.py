"""
Quiz API routes for quiz management and student submissions.
"""

import logging
from datetime import datetime
from typing import List, Optional

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from ..core.dependencies import get_db, require_auth
from ..database.models import Answer, Question, Quiz, QuizAttempt
from .schemas import (
    AnswerSubmitRequest,
    GradeAnswerRequest,
    GradingQueueItem,
    QuestionCreateRequest,
    QuestionResponse,
    QuestionResponseBase,
    QuizAttemptDetailResponse,
    QuizAttemptResponse,
    QuizCreateRequest,
    QuizDetailResponse,
    QuizResponse,
    QuizResultResponse,
    QuizStartResponse,
    QuizSubmitRequest,
    QuizUpdateRequest,
)
from .services import QuizService, ScoringService

logger = logging.getLogger(__name__)

router = APIRouter()


# ==================== STUDENT ENDPOINTS ====================


@router.get("/week/{week_number}", response_model=QuizResponse)
async def get_quiz_for_week(
    week_number: int,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Get the quiz for a specific week."""
    result = await db.execute(
        select(Quiz)
        .where(Quiz.week_number == week_number)
        .where(Quiz.is_published == True)  # noqa: E712
    )
    quiz = result.scalar_one_or_none()

    if not quiz:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"No published quiz found for week {week_number}",
        )

    questions_result = await db.execute(
        select(func.count(Question.id)).where(Question.quiz_id == quiz.id)
    )
    question_count = questions_result.scalar() or 0

    return QuizResponse(
        id=quiz.id,
        week_number=quiz.week_number,
        chapter=quiz.chapter,
        title=quiz.title,
        description=quiz.description,
        time_limit_minutes=quiz.time_limit_minutes,
        max_attempts=quiz.max_attempts,
        passing_score=quiz.passing_score,
        is_published=quiz.is_published,
        available_from=quiz.available_from,
        available_until=quiz.available_until,
        question_count=question_count,
        created_at=quiz.created_at,
    )


@router.get("/{quiz_id}", response_model=QuizDetailResponse)
async def get_quiz_detail(
    quiz_id: int,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Get full quiz details with questions (for taking the quiz)."""
    result = await db.execute(
        select(Quiz)
        .options(selectinload(Quiz.questions))
        .where(Quiz.id == quiz_id)
    )
    quiz = result.scalar_one_or_none()

    if not quiz:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Quiz not found",
        )

    if not quiz.is_published:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Quiz is not available",
        )

    attempts_count = await QuizService.get_user_attempts_count(user_id, quiz_id, db)
    best_score = await QuizService.get_user_best_score(user_id, quiz_id, db)

    questions = [
        QuestionResponseBase(
            id=q.id,
            question_type=q.question_type,
            category=q.category,
            points=q.points,
            order_index=q.order_index,
            content=QuizService.strip_correct_answers(q),
        )
        for q in sorted(quiz.questions, key=lambda x: x.order_index)
    ]

    return QuizDetailResponse(
        id=quiz.id,
        week_number=quiz.week_number,
        chapter=quiz.chapter,
        title=quiz.title,
        description=quiz.description,
        time_limit_minutes=quiz.time_limit_minutes,
        max_attempts=quiz.max_attempts,
        passing_score=quiz.passing_score,
        is_published=quiz.is_published,
        available_from=quiz.available_from,
        available_until=quiz.available_until,
        question_count=len(quiz.questions),
        created_at=quiz.created_at,
        questions=questions,
        user_attempts_count=attempts_count,
        user_best_score=best_score,
    )


@router.post("/{quiz_id}/start", response_model=QuizStartResponse)
async def start_quiz_attempt(
    quiz_id: int,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Start a new quiz attempt."""
    result = await db.execute(
        select(Quiz)
        .options(selectinload(Quiz.questions))
        .where(Quiz.id == quiz_id)
    )
    quiz = result.scalar_one_or_none()

    if not quiz:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Quiz not found",
        )

    can_start, message = await QuizService.can_start_attempt(user_id, quiz, db)
    if not can_start:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=message,
        )

    attempts_count = await QuizService.get_user_attempts_count(user_id, quiz_id, db)

    attempt = QuizAttempt(
        quiz_id=quiz_id,
        user_id=user_id,
        attempt_number=attempts_count + 1,
        started_at=datetime.utcnow(),
    )
    db.add(attempt)
    await db.commit()
    await db.refresh(attempt)

    logger.info(f"User {user_id} started quiz attempt {attempt.id} for quiz {quiz_id}")

    questions = [
        QuestionResponseBase(
            id=q.id,
            question_type=q.question_type,
            category=q.category,
            points=q.points,
            order_index=q.order_index,
            content=QuizService.strip_correct_answers(q),
        )
        for q in sorted(quiz.questions, key=lambda x: x.order_index)
    ]

    return QuizStartResponse(
        attempt_id=attempt.id,
        started_at=attempt.started_at,
        time_limit_minutes=quiz.time_limit_minutes,
        questions=questions,
    )


@router.post("/{quiz_id}/attempts/{attempt_id}/answers")
async def save_answer(
    quiz_id: int,
    attempt_id: int,
    request: AnswerSubmitRequest,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Save/update an answer for a specific question (auto-save)."""
    result = await db.execute(
        select(QuizAttempt)
        .where(QuizAttempt.id == attempt_id)
        .where(QuizAttempt.quiz_id == quiz_id)
        .where(QuizAttempt.user_id == user_id)
    )
    attempt = result.scalar_one_or_none()

    if not attempt:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Attempt not found",
        )

    if attempt.is_submitted:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Cannot modify a submitted attempt",
        )

    question_result = await db.execute(
        select(Question)
        .where(Question.id == request.question_id)
        .where(Question.quiz_id == quiz_id)
    )
    question = question_result.scalar_one_or_none()

    if not question:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Question not found in this quiz",
        )

    existing_answer = await db.execute(
        select(Answer)
        .where(Answer.attempt_id == attempt_id)
        .where(Answer.question_id == request.question_id)
    )
    answer = existing_answer.scalar_one_or_none()

    if answer:
        answer.response = request.response
    else:
        answer = Answer(
            attempt_id=attempt_id,
            question_id=request.question_id,
            response=request.response,
        )
        db.add(answer)

    await db.commit()

    return {"message": "Answer saved"}


@router.post("/{quiz_id}/attempts/{attempt_id}/submit", response_model=QuizResultResponse)
async def submit_quiz_attempt(
    quiz_id: int,
    attempt_id: int,
    request: QuizSubmitRequest,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Submit a completed quiz attempt for grading."""
    result = await db.execute(
        select(QuizAttempt)
        .where(QuizAttempt.id == attempt_id)
        .where(QuizAttempt.quiz_id == quiz_id)
        .where(QuizAttempt.user_id == user_id)
    )
    attempt = result.scalar_one_or_none()

    if not attempt:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Attempt not found",
        )

    if attempt.is_submitted:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="This attempt has already been submitted",
        )

    quiz_result = await db.execute(select(Quiz).where(Quiz.id == quiz_id))
    quiz = quiz_result.scalar_one()

    questions_result = await db.execute(
        select(Question).where(Question.quiz_id == quiz_id)
    )
    questions = {q.id: q for q in questions_result.scalars().all()}

    for answer_req in request.answers:
        if answer_req.question_id not in questions:
            continue

        existing = await db.execute(
            select(Answer)
            .where(Answer.attempt_id == attempt_id)
            .where(Answer.question_id == answer_req.question_id)
        )
        answer = existing.scalar_one_or_none()

        if answer:
            answer.response = answer_req.response
        else:
            answer = Answer(
                attempt_id=attempt_id,
                question_id=answer_req.question_id,
                response=answer_req.response,
            )
            db.add(answer)

    await db.commit()

    answers_result = await db.execute(
        select(Answer).where(Answer.attempt_id == attempt_id)
    )
    answers = list(answers_result.scalars().all())

    auto_graded_count = 0
    pending_count = 0

    for answer in answers:
        question = questions.get(answer.question_id)
        if question:
            points, status_val, feedback = ScoringService.score_answer(
                question, answer.response
            )
            answer.points_earned = points
            answer.scoring_status = status_val
            answer.feedback = feedback
            answer.graded_at = datetime.utcnow() if points is not None else None

            if status_val.value == "auto_graded":
                auto_graded_count += 1
            else:
                pending_count += 1

    attempt.is_submitted = True
    attempt.submitted_at = datetime.utcnow()

    score, max_score, percentage, is_fully_graded = await QuizService.calculate_attempt_score(
        attempt, db
    )
    attempt.score = score
    attempt.max_score = max_score
    attempt.percentage = percentage
    attempt.is_fully_graded = is_fully_graded

    await db.commit()

    logger.info(
        f"User {user_id} submitted quiz attempt {attempt_id}: {score}/{max_score} ({percentage:.1f}%)"
    )

    return QuizResultResponse(
        attempt_id=attempt.id,
        score=score,
        max_score=max_score,
        percentage=percentage,
        is_fully_graded=is_fully_graded,
        passing_score=quiz.passing_score,
        passed=percentage >= quiz.passing_score,
        auto_graded_count=auto_graded_count,
        pending_grading_count=pending_count,
    )


@router.get("/attempts", response_model=List[QuizAttemptResponse])
async def get_my_attempts(
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
    quiz_id: Optional[int] = Query(None),
):
    """Get all quiz attempts for the current user."""
    query = (
        select(QuizAttempt)
        .options(selectinload(QuizAttempt.quiz))
        .where(QuizAttempt.user_id == user_id)
    )

    if quiz_id:
        query = query.where(QuizAttempt.quiz_id == quiz_id)

    result = await db.execute(query.order_by(QuizAttempt.started_at.desc()))
    attempts = result.scalars().all()

    return [
        QuizAttemptResponse(
            id=a.id,
            quiz_id=a.quiz_id,
            quiz_title=a.quiz.title,
            attempt_number=a.attempt_number,
            started_at=a.started_at,
            submitted_at=a.submitted_at,
            score=a.score,
            max_score=a.max_score,
            percentage=a.percentage,
            is_submitted=a.is_submitted,
            is_fully_graded=a.is_fully_graded,
        )
        for a in attempts
    ]


@router.get("/attempts/{attempt_id}", response_model=QuizAttemptDetailResponse)
async def get_attempt_detail(
    attempt_id: int,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Get detailed results for a specific attempt."""
    result = await db.execute(
        select(QuizAttempt)
        .options(selectinload(QuizAttempt.quiz), selectinload(QuizAttempt.answers))
        .where(QuizAttempt.id == attempt_id)
        .where(QuizAttempt.user_id == user_id)
    )
    attempt = result.scalar_one_or_none()

    if not attempt:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Attempt not found",
        )

    if not attempt.is_submitted:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Cannot view results for an unsubmitted attempt",
        )

    questions_result = await db.execute(
        select(Question).where(Question.quiz_id == attempt.quiz_id)
    )
    questions = {q.id: q for q in questions_result.scalars().all()}

    from .schemas import AnswerResponse

    answers = [
        AnswerResponse(
            id=a.id,
            question_id=a.question_id,
            response=a.response,
            points_earned=a.points_earned,
            scoring_status=a.scoring_status,
            feedback=a.feedback,
        )
        for a in attempt.answers
    ]

    questions_with_solutions = [
        {
            "id": q.id,
            "question_type": q.question_type.value,
            "category": q.category.value,
            "points": q.points,
            "content": q.content,
        }
        for q in sorted(questions.values(), key=lambda x: x.order_index)
    ]

    return QuizAttemptDetailResponse(
        id=attempt.id,
        quiz_id=attempt.quiz_id,
        quiz_title=attempt.quiz.title,
        attempt_number=attempt.attempt_number,
        started_at=attempt.started_at,
        submitted_at=attempt.submitted_at,
        score=attempt.score,
        max_score=attempt.max_score,
        percentage=attempt.percentage,
        is_submitted=attempt.is_submitted,
        is_fully_graded=attempt.is_fully_graded,
        answers=answers,
        questions_with_solutions=questions_with_solutions,
    )


# ==================== ADMIN ENDPOINTS ====================


@router.post("/admin/quizzes", response_model=QuizResponse, status_code=status.HTTP_201_CREATED)
async def create_quiz(
    request: QuizCreateRequest,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Create a new quiz (admin only)."""
    quiz = Quiz(
        week_number=request.week_number,
        chapter=request.chapter,
        title=request.title,
        description=request.description,
        time_limit_minutes=request.time_limit_minutes,
        max_attempts=request.max_attempts,
        passing_score=request.passing_score,
        available_from=request.available_from,
        available_until=request.available_until,
    )
    db.add(quiz)
    await db.commit()
    await db.refresh(quiz)

    logger.info(f"Quiz created: {quiz.id} - {quiz.title}")

    return QuizResponse(
        id=quiz.id,
        week_number=quiz.week_number,
        chapter=quiz.chapter,
        title=quiz.title,
        description=quiz.description,
        time_limit_minutes=quiz.time_limit_minutes,
        max_attempts=quiz.max_attempts,
        passing_score=quiz.passing_score,
        is_published=quiz.is_published,
        available_from=quiz.available_from,
        available_until=quiz.available_until,
        question_count=0,
        created_at=quiz.created_at,
    )


@router.put("/admin/quizzes/{quiz_id}", response_model=QuizResponse)
async def update_quiz(
    quiz_id: int,
    request: QuizUpdateRequest,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Update quiz metadata (admin only)."""
    result = await db.execute(select(Quiz).where(Quiz.id == quiz_id))
    quiz = result.scalar_one_or_none()

    if not quiz:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Quiz not found",
        )

    update_data = request.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(quiz, field, value)

    await db.commit()
    await db.refresh(quiz)

    questions_result = await db.execute(
        select(func.count(Question.id)).where(Question.quiz_id == quiz.id)
    )
    question_count = questions_result.scalar() or 0

    return QuizResponse(
        id=quiz.id,
        week_number=quiz.week_number,
        chapter=quiz.chapter,
        title=quiz.title,
        description=quiz.description,
        time_limit_minutes=quiz.time_limit_minutes,
        max_attempts=quiz.max_attempts,
        passing_score=quiz.passing_score,
        is_published=quiz.is_published,
        available_from=quiz.available_from,
        available_until=quiz.available_until,
        question_count=question_count,
        created_at=quiz.created_at,
    )


@router.delete("/admin/quizzes/{quiz_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_quiz(
    quiz_id: int,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Delete a quiz and all related data (admin only)."""
    result = await db.execute(select(Quiz).where(Quiz.id == quiz_id))
    quiz = result.scalar_one_or_none()

    if not quiz:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Quiz not found",
        )

    await db.delete(quiz)
    await db.commit()

    logger.info(f"Quiz deleted: {quiz_id}")


@router.post("/admin/quizzes/{quiz_id}/questions", response_model=QuestionResponse)
async def add_question(
    quiz_id: int,
    request: QuestionCreateRequest,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Add a question to a quiz (admin only)."""
    result = await db.execute(select(Quiz).where(Quiz.id == quiz_id))
    quiz = result.scalar_one_or_none()

    if not quiz:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Quiz not found",
        )

    if request.order_index is None:
        count_result = await db.execute(
            select(func.count(Question.id)).where(Question.quiz_id == quiz_id)
        )
        request.order_index = (count_result.scalar() or 0) + 1

    from ..database.models import QuestionCategory as DBQuestionCategory
    from ..database.models import QuestionType as DBQuestionType

    question = Question(
        quiz_id=quiz_id,
        question_type=DBQuestionType(request.question_type.value),
        category=DBQuestionCategory(request.category.value),
        content=request.content,
        points=request.points,
        order_index=request.order_index,
    )
    db.add(question)
    await db.commit()
    await db.refresh(question)

    logger.info(f"Question added to quiz {quiz_id}: {question.id}")

    return QuestionResponse(
        id=question.id,
        question_type=request.question_type,
        category=request.category,
        points=question.points,
        order_index=question.order_index,
        content=question.content,
        created_at=question.created_at,
        updated_at=question.updated_at,
    )


@router.delete("/admin/questions/{question_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_question(
    question_id: int,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Delete a question (admin only)."""
    result = await db.execute(select(Question).where(Question.id == question_id))
    question = result.scalar_one_or_none()

    if not question:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Question not found",
        )

    await db.delete(question)
    await db.commit()

    logger.info(f"Question deleted: {question_id}")


@router.post("/admin/quizzes/{quiz_id}/publish", response_model=QuizResponse)
async def publish_quiz(
    quiz_id: int,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Publish a quiz (makes it available to students)."""
    result = await db.execute(select(Quiz).where(Quiz.id == quiz_id))
    quiz = result.scalar_one_or_none()

    if not quiz:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Quiz not found",
        )

    quiz.is_published = True
    await db.commit()
    await db.refresh(quiz)

    questions_result = await db.execute(
        select(func.count(Question.id)).where(Question.quiz_id == quiz.id)
    )
    question_count = questions_result.scalar() or 0

    logger.info(f"Quiz published: {quiz_id}")

    return QuizResponse(
        id=quiz.id,
        week_number=quiz.week_number,
        chapter=quiz.chapter,
        title=quiz.title,
        description=quiz.description,
        time_limit_minutes=quiz.time_limit_minutes,
        max_attempts=quiz.max_attempts,
        passing_score=quiz.passing_score,
        is_published=quiz.is_published,
        available_from=quiz.available_from,
        available_until=quiz.available_until,
        question_count=question_count,
        created_at=quiz.created_at,
    )


# ==================== GRADING ENDPOINTS ====================


@router.get("/admin/grading/queue", response_model=List[GradingQueueItem])
async def get_grading_queue(
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
    quiz_id: Optional[int] = Query(None),
):
    """Get answers pending manual grading (admin only)."""
    from ..database.models import ScoringStatus as DBScoringStatus

    query = (
        select(Answer)
        .options(
            selectinload(Answer.attempt).selectinload(QuizAttempt.quiz),
            selectinload(Answer.question),
        )
        .where(Answer.scoring_status == DBScoringStatus.PENDING)
    )

    if quiz_id:
        query = query.join(QuizAttempt).where(QuizAttempt.quiz_id == quiz_id)

    result = await db.execute(query)
    answers = result.scalars().all()

    return [
        GradingQueueItem(
            answer_id=a.id,
            attempt_id=a.attempt_id,
            quiz_title=a.attempt.quiz.title,
            question_text=a.question.content.get("question_text", ""),
            question_type=a.question.question_type,
            student_response=a.response,
            points_possible=a.question.points,
            submitted_at=a.attempt.submitted_at or a.attempt.started_at,
        )
        for a in answers
    ]


@router.post("/admin/answers/{answer_id}/grade")
async def grade_answer(
    answer_id: int,
    request: GradeAnswerRequest,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """Manually grade an answer (admin only)."""
    from ..database.models import ScoringStatus as DBScoringStatus

    result = await db.execute(
        select(Answer)
        .options(selectinload(Answer.attempt))
        .where(Answer.id == answer_id)
    )
    answer = result.scalar_one_or_none()

    if not answer:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Answer not found",
        )

    answer.points_earned = request.points_earned
    answer.feedback = request.feedback
    answer.scoring_status = DBScoringStatus.MANUALLY_GRADED
    answer.graded_by = user_id
    answer.graded_at = datetime.utcnow()

    score, max_score, percentage, is_fully_graded = await QuizService.calculate_attempt_score(
        answer.attempt, db
    )
    answer.attempt.score = score
    answer.attempt.max_score = max_score
    answer.attempt.percentage = percentage
    answer.attempt.is_fully_graded = is_fully_graded

    await db.commit()

    logger.info(f"Answer {answer_id} graded by user {user_id}: {request.points_earned} points")

    return {
        "message": "Answer graded successfully",
        "attempt_score": score,
        "attempt_percentage": percentage,
        "is_fully_graded": is_fully_graded,
    }
