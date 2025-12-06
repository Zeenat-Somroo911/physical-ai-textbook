import React, { useEffect } from 'react';
import { useAuth } from '../context/AuthContext';

/**
 * Protected Route Component
 * 
 * Wraps components that require authentication.
 * Redirects to sign in if user is not authenticated.
 */
export default function ProtectedRoute({ children, redirectTo = '/signin' }) {
  const { isAuthenticated, isLoading } = useAuth();

  useEffect(() => {
    if (!isLoading && !isAuthenticated()) {
      const currentPath = window.location.pathname;
      window.location.href = `${redirectTo}?redirect=${encodeURIComponent(currentPath)}`;
    }
  }, [isLoading, isAuthenticated, redirectTo]);

  if (isLoading) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '50vh',
      }}>
        <div>Loading...</div>
      </div>
    );
  }

  if (!isAuthenticated()) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '50vh',
      }}>
        <div>Redirecting to sign in...</div>
      </div>
    );
  }

  return <>{children}</>;
}

