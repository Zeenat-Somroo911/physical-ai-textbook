/**
 * Root component for Docusaurus theme.
 * 
 * This component wraps all pages and allows us to add global components
 * like the chatbot and authentication context that should be available on every page.
 */

import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import ChatBot from '../components/ChatBot';
import PersonalizeButton from '../components/PersonalizeButton';
import { AuthProvider } from '../context/AuthContext';
import { ContentPreferenceProvider } from '../context/ContentPreferenceContext';
import AuthModal from '../components/AuthModal';
import { Toaster } from 'react-hot-toast';

export default function Root({ children }) {
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false);
  const [authMode, setAuthMode] = useState('signup');
  const location = useLocation();

  // Listen for hash changes to open the auth modal
  useEffect(() => {
    const hash = location.hash;
    if (hash === '#signup') {
      setAuthMode('signup');
      setIsAuthModalOpen(true);
      // Remove hash after opening modal
      window.history.replaceState(null, '', location.pathname + location.search);
    } else if (hash === '#signin') {
      setAuthMode('signin');
      setIsAuthModalOpen(true);
      window.history.replaceState(null, '', location.pathname + location.search);
    }
  }, [location]);

  return (
    <AuthProvider>
      <ContentPreferenceProvider>
        {children}
        <ChatBot />
        <PersonalizeButton />
        <AuthModal
          isOpen={isAuthModalOpen}
          onClose={() => setIsAuthModalOpen(false)}
          initialMode={authMode}
        />
        <Toaster position="top-right" />
      </ContentPreferenceProvider>
    </AuthProvider>
  );
}
