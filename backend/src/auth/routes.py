"""
Authentication API routes for signup, signin, signout, and user management.
"""

import logging

from fastapi import APIRouter, Depends, HTTPException, Response, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..core.config import settings
from ..core.dependencies import get_db, require_auth
from ..database.models import User
from .schemas import AuthResponse, SigninRequest, SignupRequest, UserResponse
from .utils import create_access_token, hash_password, verify_password

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    request: SignupRequest,
    response: Response,
    db: AsyncSession = Depends(get_db),
):
    """
    Register a new user account.

    Args:
        request: Signup request with email, password, name
        response: FastAPI response object for setting cookies
        db: Database session

    Returns:
        AuthResponse with user data and access token

    Raises:
        409: Email already registered
        400: Invalid request data
    """
    # Check if email already exists
    stmt = select(User).where(User.email == request.email)
    result = await db.execute(stmt)
    existing_user = result.scalar_one_or_none()

    if existing_user:
        logger.warning(f"Signup attempt with existing email: {request.email}")
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered",
        )

    # Hash password
    password_hash = hash_password(request.password)

    # Create new user
    new_user = User(
        email=request.email,
        password_hash=password_hash,
        name=request.name,
        is_active=True,
    )

    db.add(new_user)
    await db.commit()
    await db.refresh(new_user)

    logger.info(f"New user created: {new_user.id} - {new_user.email}")

    # Create JWT token
    token_data = {
        "sub": new_user.id,
        "email": new_user.email,
        "name": new_user.name,
    }
    access_token = create_access_token(token_data)

    # Set httpOnly cookie
    response.set_cookie(
        key="access_token",
        value=access_token,
        httponly=True,
        secure=settings.is_production,  # HTTPS only in production
        samesite="lax",
        max_age=60 * 60 * 24 * 7,  # 7 days
    )

    # Return user data and token
    return AuthResponse(
        user=UserResponse.model_validate(new_user),
        access_token=access_token,
    )


@router.post("/signin", response_model=AuthResponse)
async def signin(
    request: SigninRequest,
    response: Response,
    db: AsyncSession = Depends(get_db),
):
    """
    Sign in an existing user.

    Args:
        request: Signin request with email and password
        response: FastAPI response object for setting cookies
        db: Database session

    Returns:
        AuthResponse with user data and access token

    Raises:
        401: Invalid credentials
    """
    # Query user by email
    stmt = select(User).where(User.email == request.email)
    result = await db.execute(stmt)
    user = result.scalar_one_or_none()

    # Verify user exists and password is correct
    # Use generic error message to prevent email enumeration
    if not user or not verify_password(request.password, user.password_hash):
        logger.warning(f"Failed signin attempt for email: {request.email}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid credentials",
        )

    # Check if user is active
    if not user.is_active:
        logger.warning(f"Signin attempt for inactive user: {user.id}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid credentials",
        )

    logger.info(f"User signed in: {user.id} - {user.email}")

    # Create JWT token
    token_data = {
        "sub": user.id,
        "email": user.email,
        "name": user.name,
    }
    access_token = create_access_token(token_data)

    # Set httpOnly cookie
    response.set_cookie(
        key="access_token",
        value=access_token,
        httponly=True,
        secure=settings.is_production,
        samesite="lax",
        max_age=60 * 60 * 24 * 7,  # 7 days
    )

    # Return user data and token
    return AuthResponse(
        user=UserResponse.model_validate(user),
        access_token=access_token,
    )


@router.post("/signout")
async def signout(response: Response):
    """
    Sign out the current user by clearing the access token cookie.

    Args:
        response: FastAPI response object for clearing cookies

    Returns:
        Success message
    """
    # Clear access_token cookie
    response.delete_cookie(key="access_token")

    logger.info("User signed out")

    return {"message": "Signed out successfully"}


@router.get("/me", response_model=UserResponse)
async def get_current_user(
    user_id: int = Depends(require_auth),
    db: AsyncSession = Depends(get_db),
):
    """
    Get current authenticated user's information.

    Args:
        user_id: Current user ID from JWT token
        db: Database session

    Returns:
        UserResponse with current user data

    Raises:
        401: Not authenticated
        404: User not found
    """
    # Query user by ID
    stmt = select(User).where(User.id == user_id)
    result = await db.execute(stmt)
    user = result.scalar_one_or_none()

    if not user:
        logger.error(f"User not found for ID from token: {user_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found",
        )

    return UserResponse.model_validate(user)
