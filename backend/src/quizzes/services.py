"""
Quiz business logic and scoring services.
"""

import re
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..database.models import (
    Answer,
    Question,
    QuestionType,
    Quiz,
    QuizAttempt,
    ScoringStatus,
)


class ScoringService:
    """Handles scoring logic for different question types."""

    @staticmethod
    def score_answer(
        question: Question,
        response: Dict,
    ) -> Tuple[Optional[float], ScoringStatus, Optional[str]]:
        """
        Score an answer based on question type.

        Returns:
            Tuple of (points_earned, scoring_status, feedback)
        """
        question_type = question.question_type
        content = question.content
        max_points = question.points

        if question_type == QuestionType.MULTIPLE_CHOICE:
            return ScoringService._score_multiple_choice(content, response, max_points)

        elif question_type == QuestionType.SHORT_ANSWER:
            return ScoringService._preliminary_score_short_answer(content, response)

        elif question_type == QuestionType.CODE_COMPLETION:
            return ScoringService._score_code_completion(content, response, max_points)

        return (None, ScoringStatus.PENDING, None)

    @staticmethod
    def _score_multiple_choice(
        content: Dict,
        response: Dict,
        max_points: float,
    ) -> Tuple[float, ScoringStatus, str]:
        """Score multiple choice: binary correct/incorrect."""
        selected = response.get("selected_option", "")
        correct_option = next(
            (opt["id"] for opt in content.get("options", []) if opt.get("is_correct")),
            None,
        )

        if selected == correct_option:
            feedback = f"Correct! {content.get('explanation', '')}"
            return (max_points, ScoringStatus.AUTO_GRADED, feedback)
        else:
            correct_text = next(
                (opt["text"] for opt in content.get("options", []) if opt.get("is_correct")),
                "",
            )
            feedback = f"Incorrect. The correct answer was: {correct_text}. {content.get('explanation', '')}"
            return (0.0, ScoringStatus.AUTO_GRADED, feedback)

    @staticmethod
    def _preliminary_score_short_answer(
        content: Dict,
        response: Dict,
    ) -> Tuple[Optional[float], ScoringStatus, Optional[str]]:
        """
        Preliminary keyword-based scoring for short answers.
        Full grading requires manual review.
        """
        text = response.get("text", "").lower()
        expected_keywords = content.get("expected_keywords", [])

        if not expected_keywords:
            return (None, ScoringStatus.PENDING, None)

        matched = sum(1 for kw in expected_keywords if kw.lower() in text)
        preliminary_feedback = (
            f"Preliminary: {matched}/{len(expected_keywords)} expected keywords found. "
            "Awaiting full review."
        )

        return (None, ScoringStatus.PENDING, preliminary_feedback)

    @staticmethod
    def _score_code_completion(
        content: Dict,
        response: Dict,
        max_points: float,
    ) -> Tuple[Optional[float], ScoringStatus, str]:
        """
        Score code completion by comparing to expected solution.
        Uses normalized string comparison.
        """
        submitted_code = response.get("code", "").strip()
        expected_solution = content.get("expected_solution", "").strip()

        def normalize(s: str) -> str:
            s = re.sub(r"\s+", " ", s)
            s = s.replace('"', "'")
            return s.lower().strip()

        submitted_normalized = normalize(submitted_code)
        expected_normalized = normalize(expected_solution)

        if submitted_normalized == expected_normalized:
            return (
                max_points,
                ScoringStatus.AUTO_GRADED,
                "Correct! Your solution matches the expected answer.",
            )

        expected_parts = [p.strip() for p in expected_solution.split(",")]
        submitted_parts = [p.strip() for p in submitted_code.split(",")]

        if len(expected_parts) == len(submitted_parts) and len(expected_parts) > 1:
            correct_parts = sum(
                1
                for e, s in zip(expected_parts, submitted_parts)
                if normalize(e) == normalize(s)
            )
            if correct_parts > 0:
                partial_score = max_points * (correct_parts / len(expected_parts))
                feedback = f"Partially correct: {correct_parts}/{len(expected_parts)} blanks correct."
                return (partial_score, ScoringStatus.AUTO_GRADED, feedback)

        return (
            None,
            ScoringStatus.PENDING,
            "Answer differs from expected. Queued for manual review.",
        )


class QuizService:
    """Quiz management and access control service."""

    @staticmethod
    async def can_start_attempt(
        user_id: int,
        quiz: Quiz,
        db: AsyncSession,
    ) -> Tuple[bool, str]:
        """Check if user can start a new quiz attempt."""
        if not quiz.is_published:
            return (False, "Quiz is not yet available")

        now = datetime.utcnow()
        if quiz.available_from and now < quiz.available_from:
            return (False, f"Quiz opens on {quiz.available_from}")
        if quiz.available_until and now > quiz.available_until:
            return (False, "Quiz deadline has passed")

        result = await db.execute(
            select(func.count(QuizAttempt.id))
            .where(QuizAttempt.quiz_id == quiz.id)
            .where(QuizAttempt.user_id == user_id)
            .where(QuizAttempt.is_submitted == True)  # noqa: E712
        )
        count = result.scalar() or 0

        if count >= quiz.max_attempts:
            return (False, f"Maximum attempts ({quiz.max_attempts}) reached")

        return (True, "")

    @staticmethod
    async def get_user_attempts_count(
        user_id: int,
        quiz_id: int,
        db: AsyncSession,
    ) -> int:
        """Get the number of submitted attempts for a user on a quiz."""
        result = await db.execute(
            select(func.count(QuizAttempt.id))
            .where(QuizAttempt.quiz_id == quiz_id)
            .where(QuizAttempt.user_id == user_id)
            .where(QuizAttempt.is_submitted == True)  # noqa: E712
        )
        return result.scalar() or 0

    @staticmethod
    async def get_user_best_score(
        user_id: int,
        quiz_id: int,
        db: AsyncSession,
    ) -> Optional[float]:
        """Get user's best percentage score across all attempts."""
        result = await db.execute(
            select(func.max(QuizAttempt.percentage))
            .where(QuizAttempt.quiz_id == quiz_id)
            .where(QuizAttempt.user_id == user_id)
            .where(QuizAttempt.is_submitted == True)  # noqa: E712
        )
        return result.scalar()

    @staticmethod
    def strip_correct_answers(question: Question) -> Dict:
        """Return question content with correct answers removed for student view."""
        content = dict(question.content)

        if question.question_type == QuestionType.MULTIPLE_CHOICE:
            content["options"] = [
                {"id": opt["id"], "text": opt["text"]}
                for opt in content.get("options", [])
            ]
            content.pop("explanation", None)

        elif question.question_type == QuestionType.SHORT_ANSWER:
            content.pop("sample_answer", None)
            content.pop("expected_keywords", None)
            content.pop("rubric", None)

        elif question.question_type == QuestionType.CODE_COMPLETION:
            content.pop("expected_solution", None)

        return content

    @staticmethod
    async def calculate_attempt_score(
        attempt: QuizAttempt,
        db: AsyncSession,
    ) -> Tuple[float, float, float, bool]:
        """
        Calculate total score for an attempt.

        Returns:
            Tuple of (score, max_score, percentage, is_fully_graded)
        """
        answers_result = await db.execute(
            select(Answer).where(Answer.attempt_id == attempt.id)
        )
        answers = list(answers_result.scalars().all())

        questions_result = await db.execute(
            select(Question).where(Question.quiz_id == attempt.quiz_id)
        )
        questions = {q.id: q for q in questions_result.scalars().all()}

        total_earned = 0.0
        total_possible = 0.0
        all_graded = True

        answered_question_ids = set()
        for answer in answers:
            question = questions.get(answer.question_id)
            if question:
                answered_question_ids.add(answer.question_id)
                total_possible += question.points
                if answer.points_earned is not None:
                    total_earned += answer.points_earned
                else:
                    all_graded = False

        for qid, question in questions.items():
            if qid not in answered_question_ids:
                total_possible += question.points

        percentage = (total_earned / total_possible * 100) if total_possible > 0 else 0

        return (total_earned, total_possible, percentage, all_graded)
