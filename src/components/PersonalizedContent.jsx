import React, { useState, useEffect } from 'react';
import PersonalizeButton from './PersonalizeButton';

/**
 * PersonalizedContent Component
 * 
 * Wrapper component that manages content personalization.
 * Use this in MDX files to enable personalization.
 * 
 * Usage in MDX:
 * ```jsx
 * import PersonalizedContent from '@site/src/components/PersonalizedContent';
 * 
 * <PersonalizedContent
 *   chapterId="module-01-ros2/01-introduction"
 *   chapterPath="/docs/module-01-ros2/01-introduction"
 * >
 *   {original content here}
 * </PersonalizedContent>
 * ```
 */
export default function PersonalizedContent({ 
  children, 
  chapterId, 
  chapterPath 
}) {
  const [content, setContent] = useState(children);
  const [isTransitioning, setIsTransitioning] = useState(false);

  // Extract text content from children
  const getTextContent = (node) => {
    if (typeof node === 'string') {
      return node;
    }
    if (Array.isArray(node)) {
      return node.map(getTextContent).join('');
    }
    if (node?.props?.children) {
      return getTextContent(node.props.children);
    }
    return '';
  };

  const originalContent = typeof children === 'string' 
    ? children 
    : getTextContent(children);

  const handleContentChange = (newContent) => {
    setIsTransitioning(true);
    
    // Smooth transition
    setTimeout(() => {
      setContent(newContent);
      setIsTransitioning(false);
    }, 300);
  };

  return (
    <div style={{ position: 'relative' }}>
      <PersonalizeButton
        chapterId={chapterId}
        chapterPath={chapterPath}
        originalContent={originalContent}
        onContentChange={handleContentChange}
      />
      
      <div
        style={{
          opacity: isTransitioning ? 0.5 : 1,
          transition: 'opacity 0.3s ease',
        }}
      >
        {typeof content === 'string' ? (
          <div dangerouslySetInnerHTML={{ __html: content }} />
        ) : (
          content
        )}
      </div>
    </div>
  );
}

