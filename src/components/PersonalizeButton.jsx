import React, { useState } from 'react';
import { useLocation, useHistory } from '@docusaurus/router';
import { useContentPreference } from '../context/ContentPreferenceContext';
import PersonalizationModal from './PersonalizationModal';
import toast from 'react-hot-toast';
import useBaseUrl from '@docusaurus/useBaseUrl';

export default function PersonalizeButton() {
  const { isActive, language, difficulty, setPreference, reset } = useContentPreference();
  const [isModalOpen, setIsModalOpen] = useState(false);
  const location = useLocation();
  const history = useHistory();

  console.log('[PersonalizeButton] Current state:', { isActive, language, difficulty, pathname: location.pathname });

  const handlePersonalize = ({ language, difficulty }) => {
    console.log('[PersonalizeButton] ===== handlePersonalize CALLED =====');
    console.log('[PersonalizeButton] Language:', language, 'Difficulty:', difficulty);

    // Save preference
    setPreference(language, difficulty);
    console.log('[PersonalizeButton] Preference saved');

    // Language mapping
    const langMap = {
      'english': 'english',
      'urdu': 'urdu',
      'roman_urdu': 'roman'
    };

    // Get current path
    const pathParts = location.pathname.split('/');
    console.log('[PersonalizeButton] Path parts:', pathParts);

    const docsIndex = pathParts.indexOf('docs');
    console.log('[PersonalizeButton] Docs index:', docsIndex);

    if (docsIndex !== -1) {
      const chapterPath = pathParts[docsIndex + 1];
      console.log('[PersonalizeButton] Chapter:', chapterPath);

      const mappedLang = langMap[language];
      const newRoute = `${difficulty}-${mappedLang}`;
      const fullRoute = `/physical-ai-textbook/docs/${chapterPath}/${newRoute}`;

      console.log('[PersonalizeButton] New route:', fullRoute);

      setIsModalOpen(false);

      toast.success(
        `Navigating to ${difficulty} ${language}...`,
        { duration: 2000 }
      );

      console.log('[PersonalizeButton] Calling history.push...');
      history.push(fullRoute);
      console.log('[PersonalizeButton] ===== Navigation called =====');
    } else {
      console.error('[PersonalizeButton] ERROR: Not on docs page!');
      toast.error('Not on a docs page');
    }
  };

  const handleReset = () => {
    console.log('[PersonalizeButton] Reset clicked');

    reset();

    const pathParts = location.pathname.split('/');
    const docsIndex = pathParts.indexOf('docs');

    if (docsIndex !== -1 && pathParts.length > docsIndex + 2) {
      const chapterPath = pathParts[docsIndex + 1];
      const baseRoute = `/physical-ai-textbook/docs/${chapterPath}`;

      toast.success('Resetting...', { duration: 2000 });
      history.push(baseRoute);
    } else {
      toast.success('Already on original!', { duration: 2000 });
    }
  };

  return (
    <>
      <div style={{
        position: 'fixed',
        bottom: '80px',
        right: '20px',
        zIndex: 998,
      }}>
        {isActive ? (
          <button
            onClick={handleReset}
            className="navbar-signup-opts"
            style={{
              background: 'transparent',
              border: '2px solid #00d4ff',
              color: '#00d4ff',
              padding: '10px 20px',
              fontSize: '14px',
              fontWeight: 600,
              cursor: 'pointer',
              borderRadius: '8px',
            }}
          >
            ↩️ Reset
          </button>
        ) : (
          <button
            onClick={() => {
              console.log('[PersonalizeButton] Button clicked - opening modal');
              setIsModalOpen(true);
            }}
            className="navbar-signup-opts"
            style={{
              background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
              border: 'none',
              color: 'white',
              padding: '12px 24px',
              fontSize: '14px',
              fontWeight: 600,
              cursor: 'pointer',
              borderRadius: '8px',
              boxShadow: '0 4px 12px rgba(102, 126, 234, 0.4)',
            }}
          >
            ✨ Expert Mode
          </button>
        )}
      </div>

      <PersonalizationModal
        isOpen={isModalOpen}
        onClose={() => {
          console.log('[PersonalizeButton] Modal closed');
          setIsModalOpen(false);
        }}
        onSubmit={handlePersonalize}
      />
    </>
  );
}
