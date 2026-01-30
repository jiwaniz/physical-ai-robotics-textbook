"""
Translation API routes.
"""

import logging
import httpx
from typing import List
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, Field

from ..core.dependencies import require_verified_email

logger = logging.getLogger(__name__)

router = APIRouter()

# Lingva Translate API (free, no API key required)
LINGVA_API_URL = "https://lingva.ml/api/v1"


class TranslateRequest(BaseModel):
    """Request to translate text."""
    text: str = Field(..., min_length=1, max_length=50000)
    source_lang: str = Field(default="en")
    target_lang: str = Field(default="ur")  # Urdu


class TranslateResponse(BaseModel):
    """Translation response."""
    original: str
    translated: str
    source_lang: str
    target_lang: str


class TranslateBatchRequest(BaseModel):
    """Request to translate multiple texts."""
    texts: List[str] = Field(..., min_length=1, max_length=50)
    source_lang: str = Field(default="en")
    target_lang: str = Field(default="ur")


class TranslateBatchResponse(BaseModel):
    """Batch translation response."""
    translations: List[str]
    source_lang: str
    target_lang: str


async def translate_text(text: str, source: str, target: str) -> str:
    """Translate text using Lingva API."""
    if not text.strip():
        return text

    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            # Lingva API endpoint
            url = f"{LINGVA_API_URL}/{source}/{target}/{text}"
            response = await client.get(url)

            if response.status_code == 200:
                data = response.json()
                return data.get("translation", text)
            else:
                logger.warning(f"Lingva API error: {response.status_code}")
                # Fallback to alternative API
                return await translate_with_mymemory(text, source, target)

    except Exception as e:
        logger.error(f"Translation error: {e}")
        # Try fallback
        return await translate_with_mymemory(text, source, target)


async def translate_with_mymemory(text: str, source: str, target: str) -> str:
    """Fallback translation using MyMemory API."""
    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            url = "https://api.mymemory.translated.net/get"
            params = {
                "q": text[:500],  # MyMemory has 500 char limit
                "langpair": f"{source}|{target}"
            }
            response = await client.get(url, params=params)

            if response.status_code == 200:
                data = response.json()
                if data.get("responseStatus") == 200:
                    return data["responseData"]["translatedText"]

            return text  # Return original if translation fails

    except Exception as e:
        logger.error(f"MyMemory translation error: {e}")
        return text


@router.post("/translate", response_model=TranslateResponse)
async def translate(
    request: TranslateRequest,
    user_id: str = Depends(require_verified_email),
):
    """
    Translate text to target language (default: Urdu).
    """
    try:
        translated = await translate_text(
            request.text,
            request.source_lang,
            request.target_lang
        )

        logger.info(f"User {user_id} translated {len(request.text)} chars to {request.target_lang}")

        return TranslateResponse(
            original=request.text,
            translated=translated,
            source_lang=request.source_lang,
            target_lang=request.target_lang,
        )

    except Exception as e:
        logger.error(f"Translation failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {str(e)}",
        )


@router.post("/translate/batch", response_model=TranslateBatchResponse)
async def translate_batch(
    request: TranslateBatchRequest,
    user_id: str = Depends(require_verified_email),
):
    """
    Translate multiple texts to target language.
    """
    try:
        translations = []
        for text in request.texts:
            translated = await translate_text(
                text,
                request.source_lang,
                request.target_lang
            )
            translations.append(translated)

        logger.info(f"User {user_id} batch translated {len(request.texts)} texts to {request.target_lang}")

        return TranslateBatchResponse(
            translations=translations,
            source_lang=request.source_lang,
            target_lang=request.target_lang,
        )

    except Exception as e:
        logger.error(f"Batch translation failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation failed: {str(e)}",
        )
