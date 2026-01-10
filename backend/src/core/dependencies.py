"""
FastAPI dependency injection utilities.
"""

from typing import AsyncGenerator, Optional

from fastapi import Depends, Header, HTTPException, status
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
    authorization: Optional[str] = Header(None),
) -> Optional[str]:
    """
    Extract current user ID from authorization header (session-based auth).

    This is a placeholder - actual implementation will use Better-Auth
    session validation in Phase 4.

    Args:
        authorization: Authorization header value

    Returns:
        User ID if authenticated, None otherwise
    """
    # TODO: Implement Better-Auth session validation in Phase 4 (US2)
    # For now, return None (unauthenticated)
    return None


async def require_auth(
    user_id: Optional[str] = Depends(get_current_user_id),
) -> str:
    """
    Require authentication for protected endpoints.

    Args:
        user_id: Current user ID from dependency

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
