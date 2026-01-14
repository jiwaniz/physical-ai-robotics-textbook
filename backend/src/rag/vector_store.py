"""
Qdrant vector store client wrapper for content embeddings.
"""

from typing import Any, Dict, List, Optional

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, PointStruct, VectorParams

from ..core.config import settings


class QdrantVectorStore:
    """Wrapper for Qdrant Cloud vector database operations."""

    def __init__(self):
        """Initialize Qdrant client connection."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=30.0,
        )
        self.collection_name = settings.qdrant_collection
        self.vector_size = 768  # Google text-embedding-004 dimension

    def create_collection(self, recreate: bool = False) -> None:
        """
        Create Qdrant collection for book content embeddings.

        Args:
            recreate: If True, delete existing collection and recreate
        """
        if recreate and self.collection_exists():
            self.client.delete_collection(collection_name=self.collection_name)

        if not self.collection_exists():
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE,
                ),
            )

    def collection_exists(self) -> bool:
        """Check if collection exists."""
        collections = self.client.get_collections()
        return any(c.name == self.collection_name for c in collections.collections)

    def upsert_points(self, points: List[PointStruct]) -> None:
        """
        Upsert embedding points to collection.

        Args:
            points: List of PointStruct with id, vector, and payload
        """
        self.client.upsert(
            collection_name=self.collection_name,
            points=points,
        )

    def search(
        self,
        query_vector: List[float],
        limit: int = 5,
        score_threshold: float = 0.7,
        filter_conditions: Optional[Dict[str, Any]] = None,
    ) -> List[models.ScoredPoint]:
        """
        Semantic search for similar content chunks.

        Args:
            query_vector: Embedding vector of the query
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score (0-1)
            filter_conditions: Optional metadata filters

        Returns:
            List of scored points with payload and similarity scores
        """
        search_params = {
            "collection_name": self.collection_name,
            "query_vector": query_vector,
            "limit": limit,
            "score_threshold": score_threshold,
        }

        if filter_conditions:
            search_params["query_filter"] = models.Filter(
                must=[
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value),
                    )
                    for key, value in filter_conditions.items()
                ]
            )

        return self.client.search(**search_params)

    def delete_points(self, point_ids: List[str]) -> None:
        """
        Delete points from collection by IDs.

        Args:
            point_ids: List of point IDs to delete
        """
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.PointIdsList(
                points=point_ids,
            ),
        )

    def get_collection_info(self) -> Dict[str, Any]:
        """Get collection information and statistics."""
        info = self.client.get_collection(collection_name=self.collection_name)
        return {
            "vectors_count": info.vectors_count,
            "points_count": info.points_count,
            "status": info.status,
            "optimizer_status": info.optimizer_status,
        }

    async def close(self):
        """Close Qdrant client connection."""
        await self.client.close()


# Singleton instance
_vector_store: Optional[QdrantVectorStore] = None


def get_vector_store() -> QdrantVectorStore:
    """Get or create singleton Qdrant vector store instance."""
    global _vector_store
    if _vector_store is None:
        _vector_store = QdrantVectorStore()
    return _vector_store
