"""Basic health check tests."""

import pytest
from fastapi.testclient import TestClient


def test_health_endpoint():
    """Test that health endpoint returns healthy status."""
    # Import here to avoid initialization issues
    from src.main import app

    client = TestClient(app)
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert "version" in data


def test_root_endpoint():
    """Test that root endpoint returns API info."""
    from src.main import app

    client = TestClient(app)
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert "message" in data
