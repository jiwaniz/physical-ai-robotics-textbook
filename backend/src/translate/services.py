"""
Translation service using LLM for English to Urdu translation.
"""

import hashlib
import logging
import os
import re
from typing import Dict, List, Optional, Tuple

from openai import OpenAI

logger = logging.getLogger(__name__)

# Groq LLM settings (same as chatbot)
GROQ_BASE_URL = "https://api.groq.com/openai/v1"
TRANSLATION_MODEL = "llama-3.3-70b-versatile"

# Maximum chunk size for translation (in characters)
MAX_CHUNK_SIZE = 3000


class TranslationService:
    """Service for translating content from English to Urdu."""

    def __init__(self):
        self.client = OpenAI(
            api_key=os.getenv("GROQ_API_KEY"),
            base_url=GROQ_BASE_URL,
        )
        # In-memory cache (use Redis for production)
        self._cache: Dict[str, str] = {}

    def _get_cache_key(self, content: str, target_language: str) -> str:
        """Generate a cache key for the content."""
        content_hash = hashlib.md5(content.encode()).hexdigest()
        return f"{target_language}:{content_hash}"

    def _extract_code_blocks(self, content: str) -> Tuple[str, List[Tuple[str, str]]]:
        """
        Extract code blocks and replace with placeholders.

        Returns:
            Tuple of (content with placeholders, list of (placeholder, original code))
        """
        code_blocks = []
        placeholder_pattern = "__CODE_BLOCK_{}_PLACEHOLDER__"

        # Match <pre><code> blocks
        def replace_pre_code(match):
            idx = len(code_blocks)
            placeholder = placeholder_pattern.format(idx)
            code_blocks.append((placeholder, match.group(0)))
            return placeholder

        # Match <pre> and <code> blocks
        patterns = [
            r'<pre[^>]*>.*?</pre>',
            r'<code[^>]*>.*?</code>',
        ]

        result = content
        for pattern in patterns:
            result = re.sub(pattern, replace_pre_code, result, flags=re.DOTALL)

        return result, code_blocks

    def _restore_code_blocks(self, content: str, code_blocks: List[Tuple[str, str]]) -> str:
        """Restore code blocks from placeholders."""
        result = content
        for placeholder, original in code_blocks:
            result = result.replace(placeholder, original)
        return result

    def _chunk_content(self, content: str) -> List[str]:
        """
        Split content into manageable chunks for translation.
        Tries to split on paragraph boundaries.
        """
        if len(content) <= MAX_CHUNK_SIZE:
            return [content]

        chunks = []
        current_chunk = ""

        # Split by common HTML block elements
        parts = re.split(r'(</(?:p|div|h[1-6]|li|tr)>)', content)

        for i in range(0, len(parts), 2):
            part = parts[i]
            # Add closing tag if exists
            if i + 1 < len(parts):
                part += parts[i + 1]

            if len(current_chunk) + len(part) > MAX_CHUNK_SIZE:
                if current_chunk:
                    chunks.append(current_chunk)
                current_chunk = part
            else:
                current_chunk += part

        if current_chunk:
            chunks.append(current_chunk)

        return chunks if chunks else [content]

    def _translate_chunk(self, chunk: str, preserve_html: bool = True) -> str:
        """Translate a single chunk of content."""
        system_prompt = """You are an expert translator specializing in English to Urdu translation.
Your task is to translate educational content about robotics and AI.

CRITICAL RULES:
1. Translate ALL English text to Urdu (using Urdu script نستعلیق)
2. PRESERVE all HTML tags exactly as they are (e.g., <h1>, <p>, <div>, <strong>, etc.)
3. PRESERVE all HTML attributes exactly (e.g., class="...", id="...", etc.)
4. PRESERVE all placeholder markers like __CODE_BLOCK_X_PLACEHOLDER__
5. Keep technical terms in English when there's no good Urdu equivalent (e.g., ROS, API, SDK)
6. Maintain the same HTML structure - don't add or remove tags
7. For technical terms that are transliterated, you can add the English in parentheses

Example:
Input: <h2>Introduction to Robotics</h2><p>Learn about <strong>ROS 2</strong> basics.</p>
Output: <h2>روبوٹکس کا تعارف</h2><p><strong>ROS 2</strong> کی بنیادی باتیں سیکھیں۔</p>

Translate naturally and fluently, not word-by-word."""

        user_message = f"""Translate the following HTML content from English to Urdu.
Keep all HTML tags and attributes intact. Only translate the text content.

Content to translate:
{chunk}

Return ONLY the translated HTML, nothing else."""

        try:
            response = self.client.chat.completions.create(
                model=TRANSLATION_MODEL,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message},
                ],
                temperature=0.3,  # Lower temperature for more consistent translations
                max_tokens=4000,
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            logger.error(f"Translation error: {e}")
            raise

    def translate(
        self,
        content: str,
        source_language: str = "en",
        target_language: str = "ur",
        preserve_html: bool = True,
        page_path: Optional[str] = None,
    ) -> Tuple[str, bool]:
        """
        Translate content from source language to target language.

        Args:
            content: HTML content to translate
            source_language: Source language code
            target_language: Target language code
            preserve_html: Whether to preserve HTML tags
            page_path: Optional page path for caching

        Returns:
            Tuple of (translated content, was_cached)
        """
        # Check cache
        cache_key = self._get_cache_key(content, target_language)
        if cache_key in self._cache:
            logger.info(f"Cache hit for translation: {page_path or 'unknown'}")
            return self._cache[cache_key], True

        # Extract code blocks to preserve them
        content_without_code, code_blocks = self._extract_code_blocks(content)

        # Chunk the content for translation
        chunks = self._chunk_content(content_without_code)
        logger.info(f"Translating {len(chunks)} chunks for {page_path or 'unknown'}")

        # Translate each chunk
        translated_chunks = []
        for i, chunk in enumerate(chunks):
            logger.info(f"Translating chunk {i + 1}/{len(chunks)}")
            translated_chunk = self._translate_chunk(chunk, preserve_html)
            translated_chunks.append(translated_chunk)

        # Combine translated chunks
        translated_content = "".join(translated_chunks)

        # Restore code blocks
        translated_content = self._restore_code_blocks(translated_content, code_blocks)

        # Cache the result
        self._cache[cache_key] = translated_content
        logger.info(f"Translation cached for: {page_path or 'unknown'}")

        return translated_content, False


# Singleton instance
_translation_service: Optional[TranslationService] = None


def get_translation_service() -> TranslationService:
    """Get or create the translation service singleton."""
    global _translation_service
    if _translation_service is None:
        _translation_service = TranslationService()
    return _translation_service
