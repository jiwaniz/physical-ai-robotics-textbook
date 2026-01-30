"""
RAG Chatbot services for document retrieval and LLM interaction.
"""

import json
import logging
import os
import uuid
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, PointStruct, VectorParams

logger = logging.getLogger(__name__)

# Collection name for the textbook embeddings
COLLECTION_NAME = "physical_ai_textbook"
EMBEDDING_MODEL = "models/text-embedding-004"
EMBEDDING_DIMENSION = 768

# Groq LLM settings
GROQ_BASE_URL = "https://api.groq.com/openai/v1"
CHAT_MODEL = "llama-3.3-70b-versatile"


class EmbeddingService:
    """Service for generating embeddings using Google's embedding model."""

    def __init__(self):
        self.client = OpenAI(
            api_key=os.getenv("GOOGLE_API_KEY"),
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

    def get_embedding(self, text: str) -> List[float]:
        """Generate embedding for a single text."""
        response = self.client.embeddings.create(
            model=EMBEDDING_MODEL,
            input=text,
        )
        return response.data[0].embedding

    def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts."""
        response = self.client.embeddings.create(
            model=EMBEDDING_MODEL,
            input=texts,
        )
        return [item.embedding for item in response.data]


class VectorStoreService:
    """Service for interacting with Qdrant vector database."""

    def __init__(self):
        self.client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
        )
        self.embedding_service = EmbeddingService()

    def ensure_collection(self):
        """Ensure the collection exists."""
        collections = self.client.get_collections().collections
        if not any(c.name == COLLECTION_NAME for c in collections):
            self.client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=EMBEDDING_DIMENSION,
                    distance=Distance.COSINE,
                ),
            )
            logger.info(f"Created collection: {COLLECTION_NAME}")

    def upsert_documents(
        self,
        documents: List[Dict],
        batch_size: int = 100,
    ):
        """Upsert documents into the vector store."""
        self.ensure_collection()

        for i in range(0, len(documents), batch_size):
            batch = documents[i : i + batch_size]
            texts = [doc["content"] for doc in batch]
            embeddings = self.embedding_service.get_embeddings(texts)

            points = [
                PointStruct(
                    id=doc.get("id", str(uuid.uuid4())),
                    vector=embedding,
                    payload={
                        "content": doc["content"],
                        "page_path": doc["page_path"],
                        "title": doc.get("title", ""),
                        "chunk_index": doc.get("chunk_index", 0),
                    },
                )
                for doc, embedding in zip(batch, embeddings)
            ]

            self.client.upsert(collection_name=COLLECTION_NAME, points=points)
            logger.info(f"Upserted batch {i // batch_size + 1}")

    def search(
        self,
        query: str,
        limit: int = 5,
        score_threshold: float = 0.5,
    ) -> List[Dict]:
        """Search for similar documents."""
        query_embedding = self.embedding_service.get_embedding(query)

        results = self.client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=limit,
            score_threshold=score_threshold,
        )

        return [
            {
                "content": hit.payload["content"],
                "page_path": hit.payload["page_path"],
                "title": hit.payload.get("title", ""),
                "score": hit.score,
            }
            for hit in results
        ]

    def delete_collection(self):
        """Delete the collection (for re-ingestion)."""
        try:
            self.client.delete_collection(collection_name=COLLECTION_NAME)
            logger.info(f"Deleted collection: {COLLECTION_NAME}")
        except Exception as e:
            logger.warning(f"Could not delete collection: {e}")


class LLMService:
    """Service for interacting with Groq LLM via OpenAI SDK."""

    def __init__(self):
        self.client = OpenAI(
            api_key=os.getenv("GROQ_API_KEY"),
            base_url=GROQ_BASE_URL,
        )

    def generate_answer(
        self,
        query: str,
        context: str,
        selected_text: Optional[str] = None,
        chat_history: Optional[List[Dict]] = None,
    ) -> str:
        """Generate an answer using RAG context."""
        system_prompt = """You are an expert AI tutor for the Physical AI & Humanoid Robotics textbook.
Your role is to help students understand concepts about ROS 2, robotics simulation, NVIDIA Isaac Sim, and Vision-Language-Action models.

Guidelines:
- Provide clear, educational explanations
- Use examples when helpful
- Reference the provided context
- If the context doesn't contain enough information, say so honestly
- Format responses with markdown for readability
- Be encouraging and supportive of learning"""

        # Build context with selected text priority
        context_parts = []
        if selected_text:
            context_parts.append(f"**User's Selected Text (Primary Focus):**\n{selected_text}")
        context_parts.append(f"**Relevant Documentation:**\n{context}")

        full_context = "\n\n".join(context_parts)

        messages = [{"role": "system", "content": system_prompt}]

        # Add chat history if available
        if chat_history:
            for msg in chat_history[-6:]:  # Last 3 exchanges
                messages.append({"role": msg["role"], "content": msg["content"]})

        # Add current query with context
        user_message = f"""Context:
{full_context}

Question: {query}

Please provide a helpful, educational answer based on the context above."""

        messages.append({"role": "user", "content": user_message})

        response = self.client.chat.completions.create(
            model=CHAT_MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=2000,
        )

        return response.choices[0].message.content

    def generate_quiz(
        self,
        content: str,
        num_questions: int = 5,
    ) -> List[Dict]:
        """Generate quiz questions from content."""
        system_prompt = """You are a quiz generator for an educational robotics textbook.
Generate multiple-choice questions that test understanding of the content.
Each question should have 4 options with exactly one correct answer.

Return a JSON array with this exact structure:
[
  {
    "question": "Question text here?",
    "options": ["Option A", "Option B", "Option C", "Option D"],
    "correct_index": 0,
    "explanation": "Brief explanation of the correct answer"
  }
]

Make questions that test conceptual understanding, not just memorization."""

        user_message = f"""Generate {num_questions} multiple-choice questions based on this content:

{content}

Return ONLY valid JSON array, no other text."""

        response = self.client.chat.completions.create(
            model=CHAT_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message},
            ],
            temperature=0.8,
            max_tokens=3000,
        )

        # Parse the JSON response
        response_text = response.choices[0].message.content
        # Clean up potential markdown code blocks
        if "```json" in response_text:
            response_text = response_text.split("```json")[1].split("```")[0]
        elif "```" in response_text:
            response_text = response_text.split("```")[1].split("```")[0]

        try:
            questions = json.loads(response_text.strip())
            return questions
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse quiz JSON: {e}")
            raise ValueError("Failed to generate valid quiz questions")


class RAGService:
    """Main RAG service combining retrieval and generation."""

    def __init__(self):
        self.vector_store = VectorStoreService()
        self.llm = LLMService()
        # In-memory conversation storage (use Redis/DB for production)
        self.conversations: Dict[str, List[Dict]] = {}

    def ask(
        self,
        query: str,
        selected_text: Optional[str] = None,
        page_path: Optional[str] = None,
        conversation_id: Optional[str] = None,
    ) -> Tuple[str, List[Dict], str]:
        """
        Process a question and return an answer with sources.

        Returns:
            Tuple of (answer, sources, conversation_id)
        """
        # Generate or use existing conversation ID
        if not conversation_id:
            conversation_id = str(uuid.uuid4())

        # Get conversation history
        chat_history = self.conversations.get(conversation_id, [])

        # Build search query - prioritize selected text if available
        search_query = query
        if selected_text:
            search_query = f"{selected_text} {query}"

        # Retrieve relevant documents
        sources = self.vector_store.search(search_query, limit=5)

        # Build context from sources
        context_parts = []
        for i, source in enumerate(sources, 1):
            context_parts.append(
                f"[Source {i}: {source['page_path']}]\n{source['content']}"
            )
        context = "\n\n".join(context_parts)

        # Generate answer
        answer = self.llm.generate_answer(
            query=query,
            context=context,
            selected_text=selected_text,
            chat_history=chat_history,
        )

        # Update conversation history
        chat_history.append({"role": "user", "content": query})
        chat_history.append({"role": "assistant", "content": answer})
        self.conversations[conversation_id] = chat_history

        return answer, sources, conversation_id

    def generate_quiz(
        self,
        page_content: str,
        page_path: str,
        num_questions: int = 5,
    ) -> Tuple[str, List[Dict]]:
        """
        Generate a quiz from page content.

        Returns:
            Tuple of (quiz_id, questions)
        """
        quiz_id = str(uuid.uuid4())
        questions = self.llm.generate_quiz(page_content, num_questions)

        # Add IDs to questions
        for i, q in enumerate(questions):
            q["id"] = i + 1

        return quiz_id, questions

    def grade_quiz(
        self,
        questions: List[Dict],
        answers: List[str],
    ) -> Tuple[int, int, float, List[str]]:
        """
        Grade quiz answers.

        Returns:
            Tuple of (score, total, percentage, feedback)
        """
        score = 0
        feedback = []

        for i, (question, answer) in enumerate(zip(questions, answers)):
            correct_index = question["correct_index"]
            correct_option = question["options"][correct_index]

            if answer == correct_option:
                score += 1
                feedback.append(f"Q{i+1}: Correct!")
            else:
                explanation = question.get("explanation", "")
                feedback.append(
                    f"Q{i+1}: Incorrect. The correct answer was: {correct_option}. {explanation}"
                )

        total = len(questions)
        percentage = (score / total * 100) if total > 0 else 0

        return score, total, percentage, feedback


# Singleton instances
_rag_service: Optional[RAGService] = None


def get_rag_service() -> RAGService:
    """Get or create the RAG service singleton."""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service
