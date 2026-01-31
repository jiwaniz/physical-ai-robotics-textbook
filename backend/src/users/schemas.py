"""
Pydantic schemas for user profile endpoints.
"""

from datetime import datetime
from enum import Enum
from typing import List, Literal, Optional

from pydantic import BaseModel, Field


class ExperienceLevel(str, Enum):
    """Experience level options for software and hardware background."""

    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class UserProfileRequest(BaseModel):
    """Request schema for creating or updating user profile."""

    software_level: ExperienceLevel = Field(
        ..., description="Software development experience level"
    )
    hardware_level: ExperienceLevel = Field(..., description="Hardware/robotics experience level")
    topics: List[str] = Field(
        default_factory=list,
        description="List of topics/technologies user is familiar with",
        examples=[["python", "ml", "robotics", "ros2", "computer_vision"]],
    )
    preferred_language: Literal["en", "ur"] = Field(
        default="en",
        description="Preferred UI language: 'en' (English) or 'ur' (Urdu)",
    )


class UserProfileResponse(BaseModel):
    """Response schema for user profile data."""

    id: int
    supabase_user_id: str
    software_level: Optional[str] = None
    hardware_level: Optional[str] = None
    topics: List[str] = Field(default_factory=list)
    preferred_language: str = "en"
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
