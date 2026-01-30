"""
Pydantic schemas for translation API.
"""

from typing import Optional

from pydantic import BaseModel, Field


class TranslateRequest(BaseModel):
    """Request model for content translation."""

    content: str = Field(
        ...,
        description="HTML content to translate",
        min_length=1,
        max_length=50000,
    )
    source_language: str = Field(
        default="en",
        description="Source language code",
    )
    target_language: str = Field(
        default="ur",
        description="Target language code",
    )
    preserve_html: bool = Field(
        default=True,
        description="Whether to preserve HTML tags in translation",
    )
    page_path: Optional[str] = Field(
        default=None,
        description="Page path for caching purposes",
    )


class TranslateResponse(BaseModel):
    """Response model for content translation."""

    translated_content: str = Field(
        ...,
        description="Translated HTML content",
    )
    source_language: str = Field(
        ...,
        description="Source language code",
    )
    target_language: str = Field(
        ...,
        description="Target language code",
    )
    cached: bool = Field(
        default=False,
        description="Whether the response was served from cache",
    )
