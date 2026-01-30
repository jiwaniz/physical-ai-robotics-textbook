import React, { useState, useEffect, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import { useAuth } from '../AuthContext';
import styles from './UrduToggle.module.css';

// Extract and preserve code blocks (HTML <pre> and <code> tags)
function extractCodeBlocks(content: string): { text: string; blocks: string[] } {
  const blocks: string[] = [];
  const text = content.replace(/<pre[\s\S]*?<\/pre>/gi, (match) => {
    blocks.push(match);
    return `__CODE_BLOCK_${blocks.length - 1}__`;
  });
  return { text, blocks };
}

// Restore code blocks after translation
function restoreCodeBlocks(text: string, blocks: string[]): string {
  let result = text;
  blocks.forEach((block, index) => {
    result = result.replace(`__CODE_BLOCK_${index}__`, block);
  });
  return result;
}

export default function UrduToggle(): JSX.Element {
  const location = useLocation();
  const { accessToken, apiBaseUrl } = useAuth();

  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [showTranslation, setShowTranslation] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string>('');

  // Cache key based on page path
  const cacheKey = `urdu_translation_${location.pathname}`;

  // Capture original content on mount and page change
  useEffect(() => {
    // Small delay to ensure page content is rendered
    const timer = setTimeout(() => {
      const article = document.querySelector('article');
      if (article) {
        setOriginalContent(article.innerHTML);
      }
    }, 300);

    // Reset state on page change
    setShowTranslation(false);
    setTranslatedContent(null);
    setError(null);

    // Check localStorage cache
    const cached = localStorage.getItem(cacheKey);
    if (cached) {
      setTranslatedContent(cached);
    }

    return () => clearTimeout(timer);
  }, [location.pathname, cacheKey]);

  // Translate content using the backend API
  const translateContent = useCallback(async () => {
    if (isTranslating) return;

    // Re-capture original content in case it wasn't set
    const article = document.querySelector('article');
    if (!article) {
      setError('Page content not found.');
      return;
    }

    const currentContent = article.innerHTML;
    if (!originalContent) {
      setOriginalContent(currentContent);
    }

    setIsTranslating(true);
    setError(null);

    try {
      // Extract code blocks to preserve them
      const { text: textToTranslate, blocks } = extractCodeBlocks(
        originalContent || currentContent,
      );

      const headers: Record<string, string> = {
        'Content-Type': 'application/json',
      };
      if (accessToken) {
        headers['Authorization'] = `Bearer ${accessToken}`;
      }

      const response = await fetch(`${apiBaseUrl}/api/translate`, {
        method: 'POST',
        headers,
        body: JSON.stringify({
          content: textToTranslate,
          source_language: 'en',
          target_language: 'ur',
          preserve_html: true,
          page_path: location.pathname,
        }),
      });

      if (!response.ok) {
        if (response.status === 429) {
          throw new Error('Translation service is busy. Please try again later.');
        }
        throw new Error('Translation failed. Please try again.');
      }

      const data = await response.json();

      // Restore code blocks
      const translatedWithCode = restoreCodeBlocks(data.translated_content, blocks);

      // Cache the translation
      localStorage.setItem(cacheKey, translatedWithCode);
      setTranslatedContent(translatedWithCode);
      setShowTranslation(true);

      // Apply to page
      article.innerHTML = translatedWithCode;
      article.classList.add('urdu-translated');
      document.documentElement.setAttribute('dir', 'rtl');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Translation failed');
    } finally {
      setIsTranslating(false);
    }
  }, [accessToken, apiBaseUrl, originalContent, location.pathname, cacheKey, isTranslating]);

  // Toggle between translated and original
  const handleToggle = () => {
    const article = document.querySelector('article');
    if (!article) return;

    if (showTranslation) {
      // Restore original
      if (originalContent) {
        article.innerHTML = originalContent;
        article.classList.remove('urdu-translated');
        document.documentElement.setAttribute('dir', 'ltr');
      }
      setShowTranslation(false);
    } else if (translatedContent) {
      // Show cached translation
      article.innerHTML = translatedContent;
      article.classList.add('urdu-translated');
      document.documentElement.setAttribute('dir', 'rtl');
      setShowTranslation(true);
    } else {
      // Fetch new translation
      translateContent();
    }
  };

  return (
    <div className={styles.urduToggleContainer}>
      <div className={styles.urduToggle}>
        <div className={styles.toggleInfo}>
          <span className={styles.infoIcon}>🇵🇰</span>
          <span className={styles.infoText}>
            {showTranslation ? 'اردو ترجمہ دکھایا جا رہا ہے' : 'Translate to Urdu'}
          </span>
        </div>

        <button
          className={`${styles.translateButton} ${showTranslation ? styles.active : ''}`}
          onClick={handleToggle}
          disabled={isTranslating}
        >
          {isTranslating ? (
            <>
              <span className={styles.spinner}></span>
              <span>Translating...</span>
            </>
          ) : showTranslation ? (
            <>
              <span>🔄</span>
              <span>Show English</span>
            </>
          ) : (
            <>
              <span>اردو</span>
              <span>Translate to Urdu</span>
            </>
          )}
        </button>
      </div>

      {error && (
        <div className={styles.errorMessage}>
          <span>⚠️</span>
          <span>{error}</span>
          <button className={styles.retryButton} onClick={translateContent}>
            Retry
          </button>
        </div>
      )}

      {showTranslation && (
        <div className={styles.translationNote}>
          <span>ℹ️</span>
          <span>Code blocks are kept in English</span>
        </div>
      )}
    </div>
  );
}
