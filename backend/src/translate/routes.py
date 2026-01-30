"""
Translation API routes.
"""

import logging

from fastapi import APIRouter, Depends, HTTPException, status

from ..core.dependencies import require_verified_email
from .schemas import TranslateRequest, TranslateResponse
from .services import get_translation_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("", response_model=TranslateResponse)
async def translate_content(
    request: TranslateRequest,
    user_id: str = Depends(require_verified_email),
):
    """
    Translate content from English to Urdu.

    - Preserves HTML structure and tags
    - Preserves code blocks in English
    - Caches translations for performance
    """
    try:
        translation_service = get_translation_service()

        translated_content, was_cached = translation_service.translate(
            content=request.content,
            source_language=request.source_language,
            target_language=request.target_language,
            preserve_html=request.preserve_html,
            page_path=request.page_path,
        )

        logger.info(
            f"User {user_id} translated content "
            f"({request.source_language} -> {request.target_language}), "
            f"cached={was_cached}, path={request.page_path}"
        )

        return TranslateResponse(
            translated_content=translated_content,
            source_language=request.source_language,
            target_language=request.target_language,
            cached=was_cached,
        )

    except Exception as e:
        error_msg = str(e)
        logger.error(f"Translation error: {error_msg}")

        # Check for rate limit errors
        if "429" in error_msg or "quota" in error_msg.lower() or "rate" in error_msg.lower():
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail="Translation service rate limit reached. Please try again in a few minutes.",
            )

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {error_msg}",
        )


@router.get("/health")
async def health_check():
    """Check if the translation service is healthy."""
    try:
        translation_service = get_translation_service()
        # Basic check - service instantiated
        return {"status": "healthy", "service": "translate"}
    except Exception as e:
        logger.error(f"Translation health check failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Service unhealthy: {str(e)}",
        )
