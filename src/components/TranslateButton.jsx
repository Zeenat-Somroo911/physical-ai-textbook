import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import { useAuth } from '../context/AuthContext';
import styles from './TranslateButton.module.css';
import toast from 'react-hot-toast';

// Fallback for when AuthContext is not available
const useAuthFallback = () => {
  try {
    return useAuth();
  } catch {
    return {
      user: null,
      isAuthenticated: () => false,
      getAuthHeaders: () => ({}),
    };
  }
};

const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-api-domain.com'
  : 'http://localhost:8000';

/**
 * TranslateButton Component
 * 
 * Button that translates chapter content to Urdu.
 * Appears alongside PersonalizeButton at the top of each chapter.
 */
export default function TranslateButton({
  chapterId,
  chapterPath,
  originalContent,
  onContentChange
}) {
  const { isAuthenticated, getAuthHeaders } = useAuthFallback();
  const [isUrdu, setIsUrdu] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [urduContent, setUrduContent] = useState(null);

  // Check if Urdu translation exists
  useEffect(() => {
    if (chapterId) {
      checkUrduTranslation();
    }
  }, [chapterId]);

  // Apply RTL when Urdu is active
  useEffect(() => {
    const article = document.querySelector('article');
    if (article) {
      if (isUrdu) {
        article.classList.add('rtl-content');
        document.documentElement.setAttribute('dir', 'rtl');
      } else {
        article.classList.remove('rtl-content');
        document.documentElement.setAttribute('dir', 'ltr');
      }
    }
  }, [isUrdu]);

  const checkUrduTranslation = async () => {
    try {
      const response = await fetch(
        `${API_URL}/translate/check?chapter_id=${encodeURIComponent(chapterId)}&target_lang=ur`,
        {
          headers: {
            ...getAuthHeaders(),
            'Content-Type': 'application/json',
          },
        }
      );

      if (response.ok) {
        const data = await response.json();
        if (data.has_translation) {
          setUrduContent(data.content);
          setIsUrdu(true);
          if (onContentChange) {
            onContentChange(data.content);
          }
        }
      }
    } catch (err) {
      // Silent fail
    }
  };

  const handleTranslate = async () => {
    setIsLoading(true);
    const loadingToast = toast.loading('Translating to Urdu...');

    try {
      const contentToTranslate = originalContent || extractPageContent();

      const response = await fetch(`${API_URL}/translate`, {
        method: 'POST',
        headers: {
          ...getAuthHeaders(),
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_path: chapterPath,
          content: contentToTranslate,
          target_language: 'ur',
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      setUrduContent(data.translated_content);
      setIsUrdu(true);

      updatePageContent(data.translated_content);

      if (onContentChange) {
        onContentChange(data.translated_content);
      }
      toast.success('Urdu translation active 🇵🇰', { id: loadingToast });

    } catch (err) {
      console.error('Translation error:', err);
      toast.error('Translation failed.', { id: loadingToast });
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = () => {
    setIsUrdu(false);
    setUrduContent(null);

    // Reset RTL
    const article = document.querySelector('article');
    if (article) {
      article.classList.remove('rtl-content');
      document.documentElement.setAttribute('dir', 'ltr');
    }

    // Reset to original
    if (onContentChange) {
      onContentChange(originalContent || extractPageContent());
    }

    window.location.reload();
    toast.success('Restored to English 🇬🇧');
  };

  /**
   * Extract text content from the current page
   */
  const extractPageContent = () => {
    const article = document.querySelector('article');
    if (article) {
      const content = article.cloneNode(true);
      const buttons = content.querySelectorAll('[class*="translate"], [class*="personalize"]');
      buttons.forEach(btn => btn.remove());
      return content.innerText || content.textContent || '';
    }
    return '';
  };

  /**
   * Update page content
   */
  const updatePageContent = (translatedContent) => {
    const article = document.querySelector('article');
    if (article && translatedContent) {
      article.setAttribute('data-translated', 'true');
      article.setAttribute('data-translated-content', translatedContent);
      article.classList.add('rtl-content');
      document.documentElement.setAttribute('dir', 'rtl');
    }
  };

  return (
    <div className={styles.translateContainer}>
      {isUrdu ? (
        <button
          className={clsx('button button--sm button--secondary', styles.compactButton)}
          onClick={handleReset}
          disabled={isLoading}
          title="Switch to English"
        >
          {isLoading ? '...' : '🇬🇧 English'}
        </button>
      ) : (
        <button
          className={clsx('button button--sm button--primary', styles.compactButton)}
          onClick={handleTranslate}
          disabled={isLoading}
          title="Translate to Urdu"
        >
          {isLoading ? '...' : '🇵🇰 Urdu'}
        </button>
      )}
    </div>
  );
}

