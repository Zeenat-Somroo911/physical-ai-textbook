---
sidebar_position: 0
---

# Personalization Example

This is an example of how to use the PersonalizeButton in your chapters.

## Method 1: Using PersonalizedContent Wrapper

```jsx
import PersonalizedContent from '@site/src/components/PersonalizedContent';

<PersonalizedContent
  chapterId="module-01-ros2/01-introduction"
  chapterPath="/docs/module-01-ros2/01-introduction"
>

# Your Chapter Title

Your chapter content here...

</PersonalizedContent>
```

## Method 2: Direct Component Usage

```jsx
import PersonalizeButton from '@site/src/components/PersonalizeButton';

<PersonalizeButton
  chapterId="module-01-ros2/01-introduction"
  chapterPath="/docs/module-01-ros2/01-introduction"
  originalContent={content}
  onContentChange={(newContent) => {
    // Handle content change
  }}
/>

# Your Chapter Content

Your content here...
```

## How It Works

1. **User clicks "Personalize for Me"**
2. **System analyzes user profile:**
   - Software experience level
   - Hardware experience
   - Programming languages known
   - Robotics background

3. **Content is adjusted:**
   - **Beginner**: Simplified explanations, more examples
   - **Intermediate**: Balanced content
   - **Advanced**: Focus on advanced concepts

4. **Content is cached** to save API costs

5. **User can reset** to original content anytime

## Benefits

- ✅ Personalized learning experience
- ✅ Cached results save API costs
- ✅ Smooth content transitions
- ✅ Works with user authentication

