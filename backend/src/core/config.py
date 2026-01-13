"""
Application configuration using Pydantic Settings.
Loads environment variables from .env file.
"""

from functools import lru_cache
from typing import List

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # Application
    environment: str = Field(default="development", description="Environment (development/staging/production)")
    debug: bool = Field(default=False, description="Debug mode")
    log_level: str = Field(default="INFO", description="Logging level")

    # Database
    database_url: str = Field(..., description="PostgreSQL database URL (Neon)")
    neon_database_url: str = Field(..., description="Neon Postgres connection string")

    # Qdrant Vector Database (Optional - for RAG features)
    qdrant_url: str = Field(default="", description="Qdrant cluster URL")
    qdrant_api_key: str = Field(default="", description="Qdrant API key")
    qdrant_collection: str = Field(default="book_content", description="Qdrant collection name")

    # OpenAI (Optional - for RAG features)
    openai_api_key: str = Field(default="", description="OpenAI API key")
    openai_org_id: str = Field(default="", description="OpenAI organization ID")

    # Better-Auth
    better_auth_secret: str = Field(..., description="Better-Auth secret key (256-bit)")
    better_auth_url: str = Field(default="http://localhost:8000", description="Better-Auth base URL")
    better_auth_trust_host: bool = Field(default=True, description="Trust host for Better-Auth")

    # Security
    allowed_hosts: List[str] = Field(
        default=["localhost", "127.0.0.1"],
        description="Allowed hosts for CORS"
    )
    cors_origins: List[str] = Field(
        default=["http://localhost:3000"],
        description="Allowed CORS origins"
    )
    session_cookie_secure: bool = Field(default=False, description="Secure session cookies (HTTPS only)")
    session_cookie_samesite: str = Field(default="lax", description="SameSite cookie attribute")
    session_max_age_days: int = Field(default=7, description="Session max age in days")

    # Rate Limiting
    rate_limit_per_minute: int = Field(default=10, description="Rate limit per user per minute")
    rate_limit_burst: int = Field(default=20, description="Rate limit burst size")

    # Caching
    enable_response_cache: bool = Field(default=True, description="Enable response caching")
    cache_ttl_seconds: int = Field(default=3600, description="Cache TTL in seconds")
    max_cache_size_mb: int = Field(default=100, description="Max cache size in MB")

    # Deployment
    render_external_url: str = Field(default="", description="Render external URL")

    @field_validator("cors_origins", "allowed_hosts", mode="before")
    @classmethod
    def parse_list(cls, v):
        """Parse comma-separated string into list."""
        if isinstance(v, str):
            return [item.strip() for item in v.split(",") if item.strip()]
        return v

    @property
    def is_production(self) -> bool:
        """Check if running in production environment."""
        return self.environment.lower() == "production"

    @property
    def is_development(self) -> bool:
        """Check if running in development environment."""
        return self.environment.lower() == "development"


@lru_cache()
def get_settings() -> Settings:
    """
    Get cached application settings.
    Uses lru_cache to ensure settings are loaded only once.
    """
    return Settings()


# Singleton instance
settings = get_settings()
