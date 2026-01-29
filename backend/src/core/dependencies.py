"""
FastAPI dependency injection utilities.
"""

import logging
from typing import AsyncGenerator, Optional

import httpx
from fastapi import Depends, Header, HTTPException, status
from jose import JWTError, jwt
from sqlalchemy.ext.asyncio import AsyncSession

from ..database.connection import get_async_db
from ..rag.vector_store import QdrantVectorStore, get_vector_store
from .config import Settings, get_settings, settings

logger = logging.getLogger(__name__)

# Cache for JWKS keys
_jwks_cache: Optional[dict] = None


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


async def verify_supabase_token_via_api(token: str) -> Optional[dict]:
    """
    Verify a Supabase JWT token by calling Supabase's user API.
    This works with both HS256 and ES256 tokens.

    Args:
        token: JWT token from Supabase

    Returns:
        User data if valid, None otherwise
    """
    if not settings.supabase_url:
        logger.warning("Supabase URL not configured")
        return None

    try:
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{settings.supabase_url}/auth/v1/user",
                headers={
                    "Authorization": f"Bearer {token}",
                    "apikey": settings.supabase_anon_key,
                },
                timeout=10.0,
            )

            if response.status_code == 200:
                user_data = response.json()
                logger.debug(f"Supabase user verified: {user_data.get('id')}")
                return user_data
            else:
                logger.error(f"Supabase token verification failed: {response.status_code} - {response.text}")
                return None
    except Exception as e:
        logger.error(f"Supabase API call failed: {e}")
        return None


def verify_supabase_token(token: str) -> Optional[dict]:
    """
    Verify a Supabase JWT token and return the payload (legacy HS256 method).
    Falls back to None if verification fails.

    Args:
        token: JWT token from Supabase

    Returns:
        Token payload if valid, None otherwise
    """
    if not settings.supabase_jwt_secret:
        logger.warning("Supabase JWT secret not configured")
        return None

    try:
        # Try HS256 first (older Supabase projects)
        payload = jwt.decode(
            token,
            settings.supabase_jwt_secret,
            algorithms=["HS256"],
            audience="authenticated",
        )
        return payload
    except JWTError as e:
        logger.debug(f"HS256 verification failed (may be ES256 token): {e}")
        return None


async def get_supabase_user_id_from_header(
    authorization: Optional[str] = Header(None),
) -> Optional[str]:
    """
    Extract Supabase user ID from JWT token in Authorization header.
    Supports both HS256 and ES256 tokens.

    Args:
        authorization: Authorization header value (Bearer <token>)

    Returns:
        User ID (UUID string) if authenticated, None otherwise
    """
    if not authorization:
        logger.debug("No authorization header")
        return None

    # Check for Bearer token format
    parts = authorization.split()
    if len(parts) != 2 or parts[0].lower() != "bearer":
        logger.debug(f"Invalid bearer format")
        return None

    token = parts[1]

    # Try HS256 verification first (faster, no network call)
    payload = verify_supabase_token(token)
    if payload:
        user_id = payload.get("sub")
        logger.debug(f"Decoded Supabase user_id (HS256): {user_id}")
        return user_id

    # Fall back to API verification (works with ES256)
    user_data = await verify_supabase_token_via_api(token)
    if user_data:
        user_id = user_data.get("id")
        logger.debug(f"Verified Supabase user_id (API): {user_id}")
        return user_id

    return None


async def require_auth(
    user_id: Optional[str] = Depends(get_supabase_user_id_from_header),
) -> str:
    """
    Require Supabase authentication for protected endpoints.

    Args:
        user_id: Supabase user ID from JWT token

    Returns:
        User ID (UUID string) if authenticated

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


async def optional_auth(
    user_id: Optional[str] = Depends(get_supabase_user_id_from_header),
) -> Optional[str]:
    """
    Optional Supabase authentication - returns user ID if authenticated, None otherwise.

    Args:
        user_id: Supabase user ID from JWT token

    Returns:
        User ID (UUID string) if authenticated, None otherwise
    """
    return user_id
