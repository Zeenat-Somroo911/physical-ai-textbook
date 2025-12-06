import React from 'react';
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

