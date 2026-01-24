"""
Database connection and session management for Neon Postgres.
"""

from typing import AsyncGenerator

from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import Session, sessionmaker

from ..core.config import settings

# SQLAlchemy Base for models
Base = declarative_base()

# Check if using SQLite (for testing) vs PostgreSQL (for production)
_is_sqlite = settings.database_url.startswith("sqlite")

if _is_sqlite:
    # SQLite configuration (for testing) - no pool settings
    sync_engine = create_engine(
        settings.database_url,
        echo=settings.debug,
    )
    async_engine = create_async_engine(
        settings.database_url,
        echo=settings.debug,
    )
else:
    # PostgreSQL configuration (for production) - with connection pooling
    sync_engine = create_engine(
        settings.database_url.replace("postgresql://", "postgresql+psycopg2://"),
        echo=settings.debug,
        pool_pre_ping=True,
        pool_size=5,
        max_overflow=10,
    )
    async_engine = create_async_engine(
        settings.database_url.replace("postgresql://", "postgresql+asyncpg://"),
        echo=settings.debug,
        pool_pre_ping=True,
        pool_size=5,
        max_overflow=10,
    )

# Session factories
SyncSessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=sync_engine,
)

AsyncSessionLocal = async_sessionmaker(
    async_engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autocommit=False,
    autoflush=False,
)


def get_sync_db() -> Session:
    """
    Get synchronous database session (for migrations and scripts).

    Yields:
        Session: SQLAlchemy sync session
    """
    db = SyncSessionLocal()
    try:
        yield db
    finally:
        db.close()


async def get_async_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Get asynchronous database session (for FastAPI endpoints).

    Yields:
        AsyncSession: SQLAlchemy async session
    """
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()


async def init_db():
    """Initialize database tables (development only)."""
    async with async_engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


async def close_db():
    """Close database connections."""
    await async_engine.dispose()
