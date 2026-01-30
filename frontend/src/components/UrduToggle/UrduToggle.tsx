import React, { useState, useEffect, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from '../AuthContext';
import styles from './UrduToggle.module.css';

interface TranslationCache {
  [key: string]: string;
}

// Simple language detection - checks if text is primarily English
function isEnglishContent(text: string): boolean {
  // Remove code blocks for detection
  const textWithoutCode = text.replace(/```[\s\S]*?```/g, '').replace(/`[^`]+`/g, '');
  // Check for English characters vs Urdu/Arabic script
  const englishChars = (textWithoutCode.match(/[a-zA-Z]/g) || []).length;
  const urduChars = (textWithoutCode.match(/[\u0600-\u06FF\u0750-\u077F]/g) || []).length;
  // If more than 60% is English letters, consider it English
  const total = englishChars + urduChars;
  return total === 0 || englishChars / total > 0.6;
}

// Extract and preserve code blocks
function extractCodeBlocks(content: string): { text: string; blocks: string[] } {
  const blocks: string[] = [];
  const text = content.replace(/```[\s\S]*?```/g, (match) => {
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

export default function UrduToggle(): JSX.Element | null {
  const { i18n } = useDocusaurusContext();
  const currentLocale = i18n.currentLocale;
  const location = useLocation();
  const { accessToken, apiBaseUrl } = useAuth();

  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [showTranslation, setShowTranslation] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string>('');
  const [needsTranslation, setNeedsTranslation] = useState(false);

  // Cache translations in localStorage
  const cacheKey = `urdu_translation_${location.pathname}`;

  // Check if current page content needs translation
  useEffect(() => {
    if (currentLocale !== 'ur') {
      setNeedsTranslation(false);
      setShowTranslation(false);
      return;
    }

    // Get page content from the article
    const article = document.querySelector('article');
    if (!article) return;

    const content = article.innerHTML;
    setOriginalContent(content);

    // Check if content is in English
    const textContent = article.textContent || '';
    const isEnglish = isEnglishContent(textContent);
    setNeedsTranslation(isEnglish);

    // Check cache
    const cached = localStorage.getItem(cacheKey);
    if (cached) {
      setTranslatedContent(cached);
    }
  }, [currentLocale, location.pathname, cacheKey]);

  // Translate content using Claude API
  const translateContent = useCallback(async () => {
    if (!accessToken || !originalContent || isTranslating) return;

    setIsTranslating(true);
    setError(null);

    try {
      // Extract code blocks to preserve them
      const { text: textToTranslate, blocks } = extractCodeBlocks(originalContent);

      const response = await fetch(`${apiBaseUrl}/api/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${accessToken}`,
        },
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
          throw new Error('ترجمہ سروس مصروف ہے۔ براہ کرم کچھ دیر بعد دوبارہ کوشش کریں۔');
        }
        throw new Error('ترجمہ میں خرابی۔ براہ کرم دوبارہ کوشش کریں۔');
      }

      const data = await response.json();

      // Restore code blocks
      const translatedWithCode = restoreCodeBlocks(data.translated_content, blocks);

      // Cache the translation
      localStorage.setItem(cacheKey, translatedWithCode);
      setTranslatedContent(translatedWithCode);
      setShowTranslation(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Translation failed');
    } finally {
      setIsTranslating(false);
    }
  }, [accessToken, apiBaseUrl, originalContent, location.pathname, cacheKey, isTranslating]);

  // Apply translation to the page
  useEffect(() => {
    if (!showTranslation || !translatedContent) return;

    const article = document.querySelector('article');
    if (article) {
      article.innerHTML = translatedContent;
      article.classList.add('urdu-translated');
    }

    return () => {
      // Restore original content when unmounting or toggling off
      if (article && originalContent) {
        article.innerHTML = originalContent;
        article.classList.remove('urdu-translated');
      }
    };
  }, [showTranslation, translatedContent, originalContent]);

  // Toggle translation display
  const handleToggle = () => {
    if (showTranslation) {
      // Restore original
      setShowTranslation(false);
      const article = document.querySelector('article');
      if (article && originalContent) {
        article.innerHTML = originalContent;
        article.classList.remove('urdu-translated');
      }
    } else if (translatedContent) {
      // Show cached translation
      setShowTranslation(true);
    } else {
      // Fetch new translation
      translateContent();
    }
  };

  // Don't render if not in Urdu locale or content doesn't need translation
  if (currentLocale !== 'ur' || !needsTranslation) {
    return null;
  }

  return (
    <div className={styles.urduToggleContainer}>
      <div className={styles.urduToggle}>
        <div className={styles.toggleInfo}>
          <span className={styles.infoIcon}>🌐</span>
          <span className={styles.infoText}>
            {showTranslation
              ? 'اردو ترجمہ دکھایا جا رہا ہے'
              : 'یہ صفحہ انگریزی میں ہے'}
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
              <span>ترجمہ ہو رہا ہے...</span>
            </>
          ) : showTranslation ? (
            <>
              <span>🔄</span>
              <span>اصل دکھائیں</span>
            </>
          ) : (
            <>
              <span>🇵🇰</span>
              <span>اردو میں ترجمہ کریں</span>
            </>
          )}
        </button>
      </div>

      {error && (
        <div className={styles.errorMessage}>
          <span>⚠️</span>
          <span>{error}</span>
          <button
            className={styles.retryButton}
            onClick={translateContent}
          >
            دوبارہ کوشش کریں
          </button>
        </div>
      )}

      {showTranslation && (
        <div className={styles.translationNote}>
          <span>ℹ️</span>
          <span>کوڈ بلاکس انگریزی میں رکھے گئے ہیں</span>
        </div>
      )}
    </div>
  );
}
