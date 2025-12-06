import React from 'react';
import { useLocation } from '@docusaurus/router';
import TranslateButton from '@site/src/components/TranslateButton';

export default function UrduToggle() {
    const location = useLocation();

    // Logic to extract chapter ID (reused from DocItem/index.js)
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

    // Only show on chapter pages
    const isChapterPage = chapterId &&
        !chapterId.includes('intro') &&
        !chapterId.includes('getting-started') &&
        !chapterId.includes('resources') &&
        !chapterId.includes('faq') &&
        !chapterId.includes('glossary');

    if (!isChapterPage) {
        return null;
    }

    return (
        <div style={{ display: 'flex', alignItems: 'center', height: '100%', marginLeft: '10px' }}>
            <TranslateButton
                chapterId={chapterId}
                chapterPath={chapterPath}
                originalContent={null}
                onContentChange={() => { }}
            // Note: For full functionality, TranslateButton needs to interact with the page content.
            // Since it uses document.querySelector('article'), it *should* work even from Navbar.
            />
        </div>
    );
}
