"""
Authentication middleware for session validation.
Will be fully implemented in Phase 4 (User Story 2) with Better-Auth.
"""

import logging
from typing import Callable, Optional

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

logger = logging.getLogger(__name__)


class SessionValidationMiddleware(BaseHTTPMiddleware):
    """
    Middleware for validating user sessions using Better-Auth.

    NOTE: This is a placeholder implementation.
    Full Better-Auth integration will be implemented in Phase 4 (User Story 2).
    """

    def __init__(self, app, exempt_paths: Optional[list] = None):
        """
        Initialize session validation middleware.

        Args:
            app: FastAPI application
            exempt_paths: List of paths that don't require authentication
        """
        super().__init__(app)
        self.exempt_paths = exempt_paths or [
            "/",
            "/health",
            "/docs",
            "/redoc",
            "/openapi.json",
            "/api/auth/signup",
            "/api/auth/signin",
        ]

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """
        Validate session for protected endpoints.

        Args:
            request: Incoming HTTP request
            call_next: Next middleware/handler in chain

        Returns:
            Response from handler
        """
        # Check if path is exempt from authentication
        if any(request.url.path.startswith(path) for path in self.exempt_paths):
            return await call_next(request)

        # TODO: Implement Better-Auth session validation in Phase 4
        # For now, allow all requests through (development mode)
        logger.debug(
            f"Session validation (placeholder): {request.method} {request.url.path}"
        )

        # In Phase 4, this will:
        # 1. Extract session cookie from request
        # 2. Validate session with Better-Auth
        # 3. Attach user info to request.state
        # 4. Return 401 if session is invalid

        return await call_next(request)


def get_session_user_id(request: Request) -> Optional[str]:
    """
    Extract user ID from validated session.

    Args:
        request: FastAPI request object

    Returns:
        User ID if authenticated, None otherwise

    NOTE: Placeholder implementation. Will be completed in Phase 4.
    """
    # TODO: Implement in Phase 4
    # Return user ID from request.state.user set by SessionValidationMiddleware
    return getattr(request.state, "user_id", None)
