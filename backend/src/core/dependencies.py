"""
FastAPI dependency injection utilities.
"""

from typing import AsyncGenerator, Optional

from fastapi import Cookie, Depends, Header, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from ..database.connection import get_async_db
from ..rag.vector_store import get_vector_store, QdrantVectorStore
from .config import get_settings, Settings


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
    from fastapi import Cookie
    from ..auth.utils import decode_access_token

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
        return user_id
    except Exception:
        # Invalid or expired token
        return None


async def require_auth(
    user_id: Optional[int] = Depends(get_current_user_id_from_cookie),
) -> int:
    """
    Require authentication for protected endpoints.

    Args:
        user_id: Current user ID from JWT cookie

    Returns:
        User ID if authenticated

    Raises:
        HTTPException: If user is not authenticated
    """
    if user_id is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user_id
