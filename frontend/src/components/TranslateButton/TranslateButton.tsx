import React, { useState, useCallback } from 'react';
import { useAuth } from '../AuthContext';
import Link from '@docusaurus/Link';
import styles from './TranslateButton.module.css';

interface TranslateButtonProps {
  contentSelector?: string; // CSS selector for content to translate
}

export default function TranslateButton({
  contentSelector = 'article',
}: TranslateButtonProps): JSX.Element {
  const { currentUser, accessToken, apiBaseUrl } = useAuth();
  const [isTranslated, setIsTranslated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [progress, setProgress] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string | null>(null);

  const translateContent = useCallback(async () => {
    if (!accessToken) return;

    const article = document.querySelector(contentSelector);
    if (!article) {
      setError('Content not found');
      return;
    }

    setIsLoading(true);
    setError(null);
    setProgress(0);

    try {
      // Store original content for toggle back
      if (!originalContent) {
        setOriginalContent(article.innerHTML);
      }

      // Get all text nodes that need translation
      const textElements = article.querySelectorAll('p, h1, h2, h3, h4, h5, h6, li, td, th, blockquote');
      const textsToTranslate: { element: Element; text: string }[] = [];

      textElements.forEach((el) => {
        // Skip code blocks and elements already translated
        if (el.closest('pre') || el.closest('code')) return;
        const text = el.textContent?.trim();
        if (text && text.length > 0) {
          textsToTranslate.push({ element: el, text });
        }
      });

      if (textsToTranslate.length === 0) {
        setError('No content to translate');
        setIsLoading(false);
        return;
      }

      // Batch translate in chunks of 10
      const batchSize = 10;
      let translated = 0;

      for (let i = 0; i < textsToTranslate.length; i += batchSize) {
        const batch = textsToTranslate.slice(i, i + batchSize);
        const texts = batch.map((item) => item.text);

        const response = await fetch(`${apiBaseUrl}/api/translate/batch`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            Authorization: `Bearer ${accessToken}`,
          },
          body: JSON.stringify({
            texts,
            source_lang: 'en',
            target_lang: 'ur',
          }),
        });

        if (!response.ok) {
          throw new Error('Translation failed');
        }

        const data = await response.json();

        // Apply translations
        batch.forEach((item, idx) => {
          if (data.translations[idx]) {
            item.element.textContent = data.translations[idx];
          }
        });

        translated += batch.length;
        setProgress(Math.round((translated / textsToTranslate.length) * 100));
      }

      // Apply RTL styling
      article.classList.add(styles.translatedContent);
      setIsTranslated(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Translation failed');
    } finally {
      setIsLoading(false);
      setProgress(0);
    }
  }, [accessToken, apiBaseUrl, contentSelector, originalContent]);

  const revertToOriginal = useCallback(() => {
    const article = document.querySelector(contentSelector);
    if (article && originalContent) {
      article.innerHTML = originalContent;
      article.classList.remove(styles.translatedContent);
      setIsTranslated(false);
    }
  }, [contentSelector, originalContent]);

  const handleClick = () => {
    if (isTranslated) {
      revertToOriginal();
    } else {
      translateContent();
    }
  };

  return (
    <div className={styles.translateContainer}>
      <div className={styles.translateInfo}>
        <span className={styles.translateIcon}>🌐</span>
        <span>Translate this chapter to Urdu (اردو)</span>
      </div>

      {currentUser ? (
        <div>
          <button
            className={`${styles.translateButton} ${isTranslated ? styles.translateButtonActive : ''}`}
            onClick={handleClick}
            disabled={isLoading}
          >
            {isLoading ? (
              <>
                <span className={styles.loadingSpinner}></span>
                <span>Translating... {progress}%</span>
              </>
            ) : isTranslated ? (
              <>
                <span>🔄</span>
                <span>Show Original (English)</span>
              </>
            ) : (
              <>
                <span>🇵🇰</span>
                <span>Translate to Urdu</span>
              </>
            )}
          </button>

          {isLoading && (
            <div className={styles.progressBar}>
              <div className={styles.progressFill} style={{ width: `${progress}%` }}></div>
            </div>
          )}

          {error && <div className={styles.errorMessage}>{error}</div>}
        </div>
      ) : (
        <div className={styles.loginPrompt}>
          <Link to="/signin">Sign in</Link> to translate content
        </div>
      )}
    </div>
  );
}
