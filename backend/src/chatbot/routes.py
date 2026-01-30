"""
RAG Chatbot API routes.
"""

import logging
from typing import Dict, List

from fastapi import APIRouter, Depends, HTTPException, status

from ..core.dependencies import require_verified_email
from .schemas import (
    AskRequest,
    AskResponse,
    GenerateQuizRequest,
    GenerateQuizResponse,
    QuizAnswerRequest,
    QuizQuestion,
    QuizResultResponse,
    SourceDocument,
)
from .services import get_rag_service

logger = logging.getLogger(__name__)

router = APIRouter()

# In-memory quiz storage (use Redis/DB for production)
_quiz_cache: Dict[str, List[Dict]] = {}


@router.post("/ask", response_model=AskResponse)
async def ask_question(
    request: AskRequest,
    user_id: str = Depends(require_verified_email),
):
    """
    Ask a question about the textbook content.

    Supports:
    - Standard RAG queries
    - Selection-based RAG (when selected_text is provided)
    - Conversation history (when conversation_id is provided)
    """
    try:
        rag_service = get_rag_service()

        answer, sources, conversation_id = rag_service.ask(
            query=request.query,
            selected_text=request.selected_text,
            page_path=request.page_path,
            conversation_id=request.conversation_id,
        )

        source_docs = [
            SourceDocument(
                content=s["content"][:500] + "..." if len(s["content"]) > 500 else s["content"],
                page_path=s["page_path"],
                score=s["score"],
            )
            for s in sources
        ]

        logger.info(f"User {user_id} asked: {request.query[:100]}...")

        return AskResponse(
            answer=answer,
            sources=source_docs,
            conversation_id=conversation_id,
        )

    except Exception as e:
        error_msg = str(e)
        logger.error(f"Error processing question: {error_msg}")

        # Check for rate limit errors
        if "429" in error_msg or "quota" in error_msg.lower() or "rate" in error_msg.lower():
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail="AI service rate limit reached. Please try again in a few minutes.",
            )

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process question: {error_msg}",
        )


@router.post("/quiz/generate", response_model=GenerateQuizResponse)
async def generate_quiz(
    request: GenerateQuizRequest,
    user_id: str = Depends(require_verified_email),
):
    """
    Generate a quiz based on page content.
    """
    try:
        rag_service = get_rag_service()

        quiz_id, questions = rag_service.generate_quiz(
            page_content=request.page_content,
            page_path=request.page_path,
            num_questions=request.num_questions,
        )

        # Cache questions for grading
        _quiz_cache[quiz_id] = questions

        # Remove correct answers from response
        response_questions = [
            QuizQuestion(
                id=q["id"],
                question=q["question"],
                options=q["options"],
                correct_index=-1,  # Hide correct answer
            )
            for q in questions
        ]

        logger.info(f"User {user_id} generated quiz with {len(questions)} questions")

        return GenerateQuizResponse(
            quiz_id=quiz_id,
            questions=response_questions,
            page_path=request.page_path,
        )

    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error generating quiz: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to generate quiz: {str(e)}",
        )


@router.post("/quiz/submit", response_model=QuizResultResponse)
async def submit_quiz(
    request: QuizAnswerRequest,
    user_id: str = Depends(require_verified_email),
):
    """
    Submit quiz answers and get results.
    """
    if request.quiz_id not in _quiz_cache:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Quiz not found or expired",
        )

    questions = _quiz_cache[request.quiz_id]

    if len(request.answers) != len(questions):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Expected {len(questions)} answers, got {len(request.answers)}",
        )

    rag_service = get_rag_service()
    score, total, percentage, feedback = rag_service.grade_quiz(questions, request.answers)

    logger.info(f"User {user_id} submitted quiz {request.quiz_id}: {score}/{total}")

    # Clean up cache
    del _quiz_cache[request.quiz_id]

    return QuizResultResponse(
        quiz_id=request.quiz_id,
        score=score,
        total=total,
        percentage=percentage,
        feedback=feedback,
    )


@router.get("/health")
async def health_check():
    """Check if the chatbot service is healthy."""
    try:
        rag_service = get_rag_service()
        # Try to connect to Qdrant
        rag_service.vector_store.client.get_collections()
        return {"status": "healthy", "service": "chatbot"}
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Service unhealthy: {str(e)}",
        )
