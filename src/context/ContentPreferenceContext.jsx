import React, { createContext, useContext, useState, useEffect } from 'react';

/**
 * ContentPreferenceContext
 * 
 * Global context for managing content language and difficulty preferences.
 * Persists to localStorage so preferences survive page refreshes.
 */

const ContentPreferenceContext = createContext();

export function ContentPreferenceProvider({ children }) {
    const [preferences, setPreferences] = useState({
        language: 'english',
        difficulty: 'medium',
        isActive: false, // Whether custom preferences are enabled
    });

    // Load from localStorage on mount
    useEffect(() => {
        try {
            const saved = localStorage.getItem('contentPreference');
            if (saved) {
                const parsed = JSON.parse(saved);
                setPreferences(parsed);
                console.log('📚 Loaded content preferences:', parsed);
            }
        } catch (e) {
            console.error('Error loading content preferences:', e);
        }
    }, []);

    // Save to localStorage whenever preferences change
    useEffect(() => {
        try {
            localStorage.setItem('contentPreference', JSON.stringify(preferences));
            console.log('💾 Saved content preferences:', preferences);
        } catch (e) {
            console.error('Error saving content preferences:', e);
        }
    }, [preferences]);

    const setLanguage = (language) => {
        setPreferences(prev => ({ ...prev, language, isActive: true }));
    };

    const setDifficulty = (difficulty) => {
        setPreferences(prev => ({ ...prev, difficulty, isActive: true }));
    };

    const setPreference = (language, difficulty) => {
        setPreferences({
            language,
            difficulty,
            isActive: true,
        });
    };

    const reset = () => {
        setPreferences({
            language: 'english',
            difficulty: 'medium',
            isActive: false,
        });
        localStorage.removeItem('contentPreference');
        console.log('🔄 Reset content preferences');
    };

    const value = {
        ...preferences,
        setLanguage,
        setDifficulty,
        setPreference,
        reset,
    };

    return (
        <ContentPreferenceContext.Provider value={value}>
            {children}
        </ContentPreferenceContext.Provider>
    );
}

export function useContentPreference() {
    const context = useContext(ContentPreferenceContext);
    if (!context) {
        throw new Error('useContentPreference must be used within ContentPreferenceProvider');
    }
    return context;
}

export default ContentPreferenceContext;
