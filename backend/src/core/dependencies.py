"""
FastAPI dependency injection utilities.
"""

from typing import AsyncGenerator, Optional

from fastapi import Cookie, Depends, Header, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from ..database.connection import get_async_db
from ..rag.vector_store import QdrantVectorStore, get_vector_store
from .config import Settings, get_settings


async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency to get database session.

    Yields:
        AsyncSession: Database session
    """
    async for session in get_async_db():
        yield session


def get_app_settings() -> Settings:
    """
    Dependency to get application settings.

    Returns:
        Settings: Application configuration
    """
    return get_settings()


def get_qdrant() -> QdrantVectorStore:
    """
    Dependency to get Qdrant vector store.

    Returns:
        QdrantVectorStore: Vector store instance
    """
    return get_vector_store()


async def get_current_user_id(
    access_token: Optional[str] = Header(None, alias="Cookie"),
) -> Optional[int]:
    """
    Extract current user ID from JWT token in cookie.

    Args:
        access_token: JWT token from cookie

    Returns:
        User ID if authenticated, None otherwise
    """

    # Try to get token from cookie
    # Note: FastAPI's Cookie dependency is better for extracting cookies
    return None  # This will be properly handled by the actual cookie extraction below


def get_current_user_id_from_cookie(
    access_token: Optional[str] = Cookie(None),
) -> Optional[int]:
    """
    Extract current user ID from JWT token in httpOnly cookie.

    Args:
        access_token: JWT token from cookie

    Returns:
        User ID if authenticated, None otherwise
    """
    if not access_token:
        return None

    try:
        from ..auth.utils import decode_access_token

        payload = decode_access_token(access_token)
        user_id = payload.get("sub")
        # Convert string to int (JWT sub claim is stored as string)
        return int(user_id) if user_id else None
    except Exception:
        # Invalid or expired token
        return None


def get_current_user_id_from_header(
    authorization: Optional[str] = Header(None),
) -> Optional[int]:
    """
    Extract current user ID from JWT token in Authorization header.

    Args:
        authorization: Authorization header value (Bearer <token>)

    Returns:
        User ID if authenticated, None otherwise
    """
    import logging

    logger = logging.getLogger(__name__)
    logger.info(
        f"Authorization header received: {authorization[:50] if authorization else 'None'}..."
    )

    if not authorization:
        logger.info("No authorization header")
        return None

    # Check for Bearer token format
    parts = authorization.split()
    if len(parts) != 2 or parts[0].lower() != "bearer":
        logger.info(f"Invalid bearer format: {parts}")
        return None

    token = parts[1]

    try:
        from ..auth.utils import decode_access_token

        payload = decode_access_token(token)
        user_id = payload.get("sub")
        logger.info(f"Decoded user_id from header: {user_id}")
        # Convert string to int (JWT sub claim is stored as string)
        return int(user_id) if user_id else None
    except Exception as e:
        logger.error(f"Token decode error: {e}")
        # Invalid or expired token
        return None


async def require_auth(
    cookie_user_id: Optional[int] = Depends(get_current_user_id_from_cookie),
    header_user_id: Optional[int] = Depends(get_current_user_id_from_header),
) -> int:
    """
    Require authentication for protected endpoints.
    Supports both cookie-based and header-based (Bearer token) authentication.

    Args:
        cookie_user_id: Current user ID from JWT cookie
        header_user_id: Current user ID from Authorization header

    Returns:
        User ID if authenticated

    Raises:
        HTTPException: If user is not authenticated
    """
    # Try header first, then cookie
    user_id = header_user_id or cookie_user_id

    if user_id is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user_id
