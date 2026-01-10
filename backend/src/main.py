"""
FastAPI main application entry point.
"""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .core.config import settings
from .core.exceptions import AppException
from .core.middleware import ErrorHandlingMiddleware, RequestLoggingMiddleware
from .database.connection import close_db, init_db

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.log_level),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager.
    Handles startup and shutdown events.
    """
    # Startup
    logger.info("Starting up application...")
    if settings.is_development:
        # Initialize database tables in development
        await init_db()
    logger.info(f"Application started in {settings.environment} mode")

    yield

    # Shutdown
    logger.info("Shutting down application...")
    await close_db()
    logger.info("Application shut down complete")


# Create FastAPI application
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for RAG-powered educational platform",
    version="0.1.0",
    docs_url="/docs" if settings.debug else None,
    redoc_url="/redoc" if settings.debug else None,
    lifespan=lifespan,
)

# Configure CORS for GitHub Pages frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"],
)

# Add request logging and error handling middleware
app.add_middleware(RequestLoggingMiddleware)
app.add_middleware(ErrorHandlingMiddleware)


# Global exception handler
@app.exception_handler(AppException)
async def app_exception_handler(request: Request, exc: AppException):
    """Handle custom application exceptions."""
    logger.error(
        f"Application error: {exc.message}",
        extra={
            "path": request.url.path,
            "method": request.method,
            "status_code": exc.status_code,
            "details": exc.details,
        },
    )
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.message,
            "details": exc.details,
        },
    )


# Health check endpoint
@app.get("/health", status_code=status.HTTP_200_OK, tags=["Health"])
async def health_check():
    """
    Health check endpoint for monitoring.

    Returns:
        dict: Status and version information
    """
    return {
        "status": "healthy",
        "environment": settings.environment,
        "version": "0.1.0",
    }


# Root endpoint
@app.get("/", tags=["Root"])
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI & Humanoid Robotics Textbook API",
        "docs": "/docs" if settings.debug else None,
        "health": "/health",
    }


# TODO: Import and include routers in later phases
# Phase 4 (US2): from .auth.routes import router as auth_router
# Phase 4 (US2): from .users.routes import router as users_router
# Phase 5 (US3): from .chat.routes import router as chat_router
# Phase 8 (US5): from .personalization.routes import router as personalization_router

# app.include_router(auth_router, prefix="/api/auth", tags=["Authentication"])
# app.include_router(users_router, prefix="/api/users", tags=["Users"])
# app.include_router(chat_router, prefix="/api/chat", tags=["Chat"])
# app.include_router(personalization_router, prefix="/api/personalization", tags=["Personalization"])
