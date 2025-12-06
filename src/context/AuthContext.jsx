import React, { createContext, useContext, useState, useEffect } from 'react';
import Cookies from 'js-cookie';

import toast from 'react-hot-toast';

const AuthContext = createContext(null);

export function AuthProvider({ children }) {
  const [user, setUser] = useState(null);
  const [token, setToken] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  // Load user from Cookies on mount
  useEffect(() => {
    const storedToken = Cookies.get('auth_token');
    const storedUser = Cookies.get('user');

    console.log('[AuthContext] Initializing...', { storedToken, storedUser });

    if (storedToken && storedUser) {
      try {
        setToken(storedToken);
        setUser(JSON.parse(storedUser));
        console.log('[AuthContext] User loaded from cookies');
      } catch (error) {
        console.error('[AuthContext] Error parsing user cookie:', error);
        Cookies.remove('auth_token');
        Cookies.remove('user');
      }
    } else {
      console.log('[AuthContext] No session found');
    }

    setIsLoading(false);
  }, []);

  const login = async (userData, authToken) => {
    console.log('[AuthContext] Login called', userData);
    setUser(userData);
    setToken(authToken);
    Cookies.set('auth_token', authToken, { expires: 7 }); // 7 days
    Cookies.set('user', JSON.stringify(userData), { expires: 7 });
  };

  const logout = () => {
    console.log('[AuthContext] Logout called');
    setUser(null);
    setToken(null);
    Cookies.remove('auth_token');
    Cookies.remove('user');
    Cookies.remove('chatbot_messages');
    Cookies.remove('chatbot_conversation_id');
    toast.success('Successfully logged out! See you soon. 👋', {
      duration: 3000,
      style: {
        background: '#333',
        color: '#fff',
      },
    });
  };

  const updateUser = (userData) => {
    setUser(userData);
    Cookies.set('user', JSON.stringify(userData), { expires: 7 });
  };

  const isAuthenticated = () => {
    return !!user && !!token;
  };

  const getAuthHeaders = () => {
    if (!token) return {};
    return {
      'Authorization': `Bearer ${token}`,
    };
  };

  const value = {
    user,
    token,
    isLoading,
    login,
    logout,
    updateUser,
    isAuthenticated,
    getAuthHeaders,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

