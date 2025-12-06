import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import { useAuth } from '../context/AuthContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './PersonalizeButton.module.css';
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
 * Extract text content from the current page
 * Used when originalContent is not provided
 */
const extractPageContent = () => {
  const article = document.querySelector('article');
  if (article) {
    // Get all text content, excluding the button itself
    const content = article.cloneNode(true);
    const buttons = content.querySelectorAll('[class*="personalize"]');
    buttons.forEach(btn => btn.remove());
    return content.innerText || content.textContent || '';
  }
  return '';
};

/**
 * PersonalizeButton Component
 * 
 * Button that personalizes chapter content based on user profile.
 * Appears at the top of each chapter page.
 */
export default function PersonalizeButton({
  chapterId,
  chapterPath,
  originalContent,
  onContentChange
}) {
  const { user, isAuthenticated, getAuthHeaders } = useAuthFallback();
  const signinBaseUrl = useBaseUrl('/signin');
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  // Get content - use provided or extract from page
  const getContent = () => {
    if (originalContent) {
      return originalContent;
    }
    return extractPageContent();
  };

  // Check if personalized content exists
  useEffect(() => {
    if (isAuthenticated() && chapterId) {
      checkPersonalizedContent();
    }
  }, [chapterId, isAuthenticated]);

  const checkPersonalizedContent = async () => {
    try {
      const response = await fetch(
        `${API_URL}/personalize/check?chapter_id=${encodeURIComponent(chapterId)}`,
        {
          headers: {
            ...getAuthHeaders(),
            'Content-Type': 'application/json',
          },
        }
      );

      if (response.ok) {
        const data = await response.json();
        if (data.has_personalized) {
          setIsPersonalized(true);
          if (onContentChange) {
            onContentChange(data.content);
          }
        }
      }
    } catch (err) {
      console.error('Error checking personalized content:', err);
    }
  };

  const handlePersonalize = async () => {
    if (!isAuthenticated()) {
      toast.error('Please Sign In to use Expert Mode', {
        style: { background: '#333', color: '#fff' },
        icon: '🔒',
      });
      // Optional: trigger auth modal if possible, or redirect
      return;
    }

    setIsLoading(true);
    const loadingToast = toast.loading('Personalizing content for you...');

    try {
      const contentToPersonalize = getContent();

      const response = await fetch(`${API_URL}/personalize`, {
        method: 'POST',
        headers: {
          ...getAuthHeaders(),
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_path: chapterPath,
          content: contentToPersonalize,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Personalization failed');
      }

      const data = await response.json();
      setIsPersonalized(true);

      if (onContentChange) {
        onContentChange(data.personalized_content);
      }
      toast.success('Content personalized! 🚀', { id: loadingToast });

    } catch (err) {
      console.error('Personalization error:', err);
      toast.error(err.message || 'Failed to personalize.', { id: loadingToast });
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = async () => {
    setIsLoading(true);
    const loadingToast = toast.loading('Restoring original content...');

    try {
      const response = await fetch(`${API_URL}/personalize/reset`, {
        method: 'POST',
        headers: {
          ...getAuthHeaders(),
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to reset content');
      }

      setIsPersonalized(false);

      if (onContentChange) {
        onContentChange(getContent());
      }
      toast.success('Original content restored', { id: loadingToast });

    } catch (err) {
      console.error('Reset error:', err);
      toast.error('Failed to reset content.', { id: loadingToast });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.personalizeContainer}>
      {isPersonalized ? (
        <button
          className={clsx('button button--sm button--secondary', styles.compactButton)}
          onClick={handleReset}
          disabled={isLoading}
          title="Reset to Original"
        >
          {isLoading ? '...' : '↩️ Reset'}
        </button>
      ) : (
        <button
          className={clsx('button button--sm button--primary', styles.compactButton)}
          onClick={handlePersonalize}
          disabled={isLoading}
          title="Personalize Content"
        >
          {isLoading ? '...' : '✨ Expert Mode'}
        </button>
      )}
    </div>
  );
}

