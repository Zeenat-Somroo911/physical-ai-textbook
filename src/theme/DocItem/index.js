import React, { useState, useEffect } from 'react';
import DocItem from '@theme-original/DocItem';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslateButton from '@site/src/components/TranslateButton';
import { useLocation } from '@docusaurus/router';

/**
 * Custom DocItem wrapper that injects PersonalizeButton at the top of each chapter.
 * 
 * This automatically adds personalization to all doc pages without modifying
 * individual markdown files.
 * 
 * The button will:
 * 1. Extract content from the page automatically
 * 2. Personalize based on user profile
 * 3. Cache results to save API costs
 * 4. Allow reset to original content
 */
export default function DocItemWrapper(props) {
  const location = useLocation();
  const [personalizedContent, setPersonalizedContent] = useState(null);
  const [translatedContent, setTranslatedContent] = useState(null);

  // Load persisted personalization from localStorage on mount/navigation
  useEffect(() => {
    const chapterId = getChapterId();
    if (!chapterId) return;

    // Check localStorage for saved personalization
    const saved = localStorage.getItem(`personalized_${chapterId}`);
    if (saved) {
      try {
        const data = JSON.parse(saved);
        setPersonalizedContent(data.content);
        console.log('Loaded personalized content from cache for:', chapterId);
      } catch (e) {
        console.error('Failed to load personalized content:', e);
      }
    } else {
      // Clear state if no saved version
      setPersonalizedContent(null);
      setTranslatedContent(null);
    }
  }, [location.pathname]);

  // Extract chapter ID from path
  // Example: /docs/module-01-ros2/introduction -> module-01-ros2/introduction
  const getChapterId = () => {
    const path = location.pathname;
    // Match /docs/... paths
    const match = path.match(/\/docs\/(.+)/);
    if (match) {
      return match[1];
    }
    return null;
  };

  const chapterId = getChapterId();
  const chapterPath = location.pathname;

  // Show buttons on all doc pages
  const isChapterPage = !!chapterId;

  const handlePersonalizedContent = (newContent) => {
    console.log('🔍 handlePersonalizedContent called with:', newContent ? `Content length: ${newContent.length}` : 'null');
    const chapterId = getChapterId();
    console.log('🔍 Chapter ID:', chapterId);

    if (newContent === null) {
      // Reset: clear localStorage and reload
      console.log('🔍 Reset mode - clearing localStorage');
      if (chapterId) {
        localStorage.removeItem(`personalized_${chapterId}`);
        console.log('✅ Removed from localStorage:', `personalized_${chapterId}`);
      }
      setPersonalizedContent(null);
      setTranslatedContent(null);
      window.location.reload();
    } else {
      // Save to localStorage for persistence
      console.log('🔍 Personalize mode - saving to localStorage');
      if (chapterId) {
        const key = `personalized_${chapterId}`;
        const data = JSON.stringify({
          content: newContent,
          timestamp: Date.now()
        });
        console.log('🔍 Attempting localStorage.setItem with key:', key);
        console.log('🔍 Data length:', data.length);
        try {
          localStorage.setItem(key, data);
          console.log('✅ Successfully saved to localStorage!');
          // Verify it was saved
          const verify = localStorage.getItem(key);
          console.log('🔍 Verification - read back from localStorage:', verify ? 'SUCCESS' : 'FAILED');
        } catch (e) {
          console.error('❌ localStorage.setItem FAILED:', e);
        }
      } else {
        console.error('❌ No chapterId - cannot save to localStorage');
      }
      setPersonalizedContent(newContent);
      setTranslatedContent(null);
    }
  };

  const handleTranslatedContent = (newContent) => {
    const chapterId = getChapterId();

    if (newContent === null) {
      setTranslatedContent(null);
      setPersonalizedContent(null);
      window.location.reload();
    } else {
      setTranslatedContent(newContent);
      setPersonalizedContent(null);
    }
  };

  // If we have custom content (non-null string), render it instead of original
  const hasCustomContent = (personalizedContent && typeof personalizedContent === 'string' && personalizedContent.length > 0) ||
    (translatedContent && typeof translatedContent === 'string' && translatedContent.length > 0);

  if (hasCustomContent && isChapterPage) {
    return (
      <>
        <div style={{
          marginBottom: '2rem',
          position: 'sticky',
          top: '60px',
          zIndex: 10,
          backgroundColor: 'var(--ifm-background-color)',
          padding: '1rem 0',
          borderBottom: '1px solid var(--ifm-color-emphasis-200)'
        }}>
          <div style={{ display: 'flex', gap: '0.75rem', flexWrap: 'wrap' }}>
            <PersonalizeButton
              chapterId={chapterId}
              chapterPath={chapterPath}
              onContentChange={handlePersonalizedContent}
            />
            <TranslateButton
              chapterId={chapterId}
              chapterPath={chapterPath}
              onContentChange={handleTranslatedContent}
            />
          </div>
        </div>

        <article style={{
          background: personalizedContent
            ? 'linear-gradient(to right, rgba(37, 99, 235, 0.03), rgba(59, 130, 246, 0.03))'
            : 'linear-gradient(to right, rgba(139, 92, 246, 0.03), rgba(168, 85, 247, 0.03))',
          borderLeft: personalizedContent ? '3px solid #3b82f6' : '3px solid #a855f7',
          paddingLeft: '1.5rem',
          transition: 'all 0.3s ease'
        }}>
          <div style={{
            display: 'inline-block',
            background: personalizedContent
              ? 'linear-gradient(135deg, #3b82f6 0%, #2563eb 100%)'
              : 'linear-gradient(135deg, #a855f7 0%, #9333ea 100%)',
            color: 'white',
            padding: '0.4rem 1rem',
            borderRadius: '16px',
            fontSize: '0.85rem',
            fontWeight: 600,
            marginBottom: '1rem',
            boxShadow: '0 2px 6px rgba(37, 99, 235, 0.4)'
          }}>
            {personalizedContent ? '✨ Personalized for You' : '🇵🇰 Urdu Translation'}
          </div>
          <div
            className="markdown"
            dangerouslySetInnerHTML={{
              __html: (personalizedContent || translatedContent).replace(/\n/g, '<br/>')
            }}
          />
        </article>
      </>
    );
  }

  return (
    <>
      {isChapterPage && (
        <div style={{
          marginBottom: '2rem',
          position: 'sticky',
          top: '60px',
          zIndex: 10,
          backgroundColor: 'var(--ifm-background-color)',
          padding: '1rem 0',
          borderBottom: '1px solid var(--ifm-color-emphasis-200)'
        }}>
          <div style={{
            display: 'flex',
            gap: '0.75rem',
            flexWrap: 'wrap',
            alignItems: 'center'
          }}>
            <PersonalizeButton
              chapterId={chapterId}
              chapterPath={chapterPath}
              originalContent={null} // Will be extracted from page
              onContentChange={(newContent) => {
                // Content update will be handled by the button
                // This can be extended for custom behavior
              }}
            />
            <TranslateButton
              chapterId={chapterId}
              chapterPath={chapterPath}
              originalContent={null} // Will be extracted from page
              onContentChange={(newContent) => {
                // Content update will be handled by the button
                // This can be extended for custom behavior
              }}
            />
          </div>
        </div>
      )}
      <DocItem {...props} />
    </>
  );
}

