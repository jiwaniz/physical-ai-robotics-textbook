"""
User profile API routes for managing user background and preferences.
"""

import logging

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..core.dependencies import get_db, require_auth
from ..database.models import UserProfile
from .schemas import UserProfileRequest, UserProfileResponse

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/profile", response_model=UserProfileResponse)
async def create_or_update_profile(
    request: UserProfileRequest,
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """
    Create or update the current user's profile.

    This endpoint performs an upsert operation:
    - If profile exists, it updates it
    - If profile doesn't exist, it creates a new one

    Args:
        request: Profile data with software_level, hardware_level, topics
        user_id: Current user ID from JWT token
        db: Database session

    Returns:
        UserProfileResponse with updated profile data
    """
    # Check if profile already exists
    stmt = select(UserProfile).where(UserProfile.user_id == user_id)
    result = await db.execute(stmt)
    existing_profile = result.scalar_one_or_none()

    if existing_profile:
        # Update existing profile
        existing_profile.software_level = request.software_level.value
        existing_profile.hardware_level = request.hardware_level.value
        existing_profile.topics = request.topics

        await db.commit()
        await db.refresh(existing_profile)

        logger.info(f"Profile updated for user: {user_id}")
        return UserProfileResponse.model_validate(existing_profile)
    else:
        # Create new profile
        new_profile = UserProfile(
            user_id=user_id,
            software_level=request.software_level.value,
            hardware_level=request.hardware_level.value,
            topics=request.topics,
        )

        db.add(new_profile)
        await db.commit()
        await db.refresh(new_profile)

        logger.info(f"Profile created for user: {user_id}")
        return UserProfileResponse.model_validate(new_profile)


@router.get("/profile", response_model=UserProfileResponse)
async def get_profile(
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """
    Get the current user's profile.

    Args:
        user_id: Current user ID from JWT token
        db: Database session

    Returns:
        UserProfileResponse with profile data

    Raises:
        404: Profile not found (user hasn't completed onboarding)
    """
    # Query profile by user_id
    stmt = select(UserProfile).where(UserProfile.user_id == user_id)
    result = await db.execute(stmt)
    profile = result.scalar_one_or_none()

    if not profile:
        logger.info(f"Profile not found for user: {user_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found. Please complete onboarding.",
        )

    return UserProfileResponse.model_validate(profile)
