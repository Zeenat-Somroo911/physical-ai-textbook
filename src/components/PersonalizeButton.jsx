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
      console.log(`Sending content for personalization: ${contentToPersonalize.length} chars`);

      // Add timeout to prevent infinite hanging (120s for AI processing)
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 120000); // 120s timeout

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
        signal: controller.signal
      });
      clearTimeout(timeoutId);

      if (!response.ok) {
        let errorData;
        try {
          const text = await response.text();
          console.error('Backend Error Response:', text);
          errorData = JSON.parse(text);
        } catch (e) {
          errorData = { detail: `Status ${response.status}: ${response.statusText}` };
        }
        throw new Error(errorData.detail || 'Personalization failed');
      }

      const data = await response.json();
      console.log('🎯 Personalization API Response:', {
        hasPersonalizedContent: !!data.personalized_content,
        contentLength: data.personalized_content?.length,
        hasOnContentChangeCallback: !!onContentChange
      });
      setIsPersonalized(true);

      if (onContentChange) {
        console.log('🎯 Calling onContentChange with personalized content...');
        onContentChange(data.personalized_content);
        console.log('✅ onContentChange called successfully');
      } else {
        console.error('❌ onContentChange callback is not defined!');
      }

      // Add visual indicator to article
      setTimeout(() => {
        const article = document.querySelector('article');
        if (article) {
          article.style.background = 'linear-gradient(to right, rgba(37, 99, 235, 0.03), rgba(59, 130, 246, 0.03))';
          article.style.borderLeft = '3px solid #3b82f6';
          article.style.paddingLeft = '1.5rem';
          article.style.transition = 'all 0.3s ease';

          // Add "Personalized for You" badge
          if (!article.querySelector('.personalized-badge')) {
            const badge = document.createElement('div');
            badge.className = 'personalized-badge';
            badge.style.cssText = `
              display: inline-block;
              background: linear-gradient(135deg, #3b82f6 0%, #2563eb 100%);
              color: white;
              padding: 0.4rem 1rem;
              border-radius: 16px;
              font-size: 0.85rem;
              font-weight: 600;
              margin-bottom: 1rem;
              box-shadow: 0 2px 6px rgba(37, 99, 235, 0.4);
            `;
            badge.textContent = '✨ Personalized for You';
            article.insertBefore(badge, article.firstChild);
          }
        }
      }, 100);

      toast.success('Content personalized! 🚀', { id: loadingToast });

    } catch (err) {
      console.error('Personalization error DETAILS:', err);
      // Attempt to read parsing error response
      let errorMsg = err.message || 'Failed to personalize.';

      // Special handling for timeout
      if (err.name === 'AbortError') {
        errorMsg = 'Request timed out. Content might be too large or server is slow. Please try again.';
      }

      toast.error(`Error: ${errorMsg}`, { id: loadingToast });
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

      // Clear personalized content to show original
      if (onContentChange) {
        onContentChange(null);
      }

      toast.success('Original content restored', { id: loadingToast });

      // Hard reload to completely clear all state and styling
      setTimeout(() => {
        window.location.reload(true);
      }, 800);

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

