---
sidebar_position: 0
---

# Content Personalization Guide

Learn how to add personalization to your chapters.

## Quick Start

Add this to the top of any chapter MDX file:

```jsx
import PersonalizeButton from '@site/src/components/PersonalizeButton';

<PersonalizeButton
  chapterId="module-01-ros2/01-introduction"
  chapterPath="/docs/module-01-ros2/01-introduction"
  originalContent={`
# Your Chapter Content

Your markdown content here...
  `}
  onContentChange={(newContent) => {
    // Content will be updated automatically
  }}
/>

# Your Chapter Title

Your content here...
```

## How Personalization Works

### For Beginners
- Simplified explanations
- More step-by-step examples
- Additional context and background
- Simpler language

### For Intermediate Users
- Standard explanations
- Balanced examples
- Mix of basics and advanced

### For Advanced Users
- Skip basic explanations
- Focus on advanced concepts
- Assume prior knowledge
- Advanced examples

## Caching

Personalized content is cached in the database to:
- Save API costs
- Provide instant loading
- Maintain consistency

Content is re-personalized only if:
- Original content changes
- User profile changes
- Cache is manually cleared

## User Experience

1. User sees "Personalize for Me" button
2. Clicks button (must be signed in)
3. Content is personalized based on their profile
4. Button changes to "Reset to Original"
5. User can toggle between personalized and original

## Integration Example

Here's a complete example for a chapter:

```jsx
import React, { useState } from 'react';
import PersonalizeButton from '@site/src/components/PersonalizeButton';

export default function ChapterContent() {
  const [content, setContent] = useState(originalContent);
  
  return (
    <>
      <PersonalizeButton
        chapterId="module-01-ros2/01-introduction"
        chapterPath="/docs/module-01-ros2/01-introduction"
        originalContent={originalContent}
        onContentChange={setContent}
      />
      
      <div dangerouslySetInnerHTML={{ __html: content }} />
    </>
  );
}
```

