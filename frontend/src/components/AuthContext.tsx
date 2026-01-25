import React, { createContext, useContext, useState, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface UserResponse {
  id: number;
  email: string;
  name: string;
  is_active: boolean;
  email_verified: boolean;
  created_at: string;
}

interface AuthContextType {
  currentUser: UserResponse | null;
  accessToken: string | null;
  apiBaseUrl: string;
  isLoading: boolean;
  error: string | null;
  signup: (email: string, password: string, name: string) => Promise<void>;
  signin: (email: string, password: string) => Promise<void>;
  signout: () => Promise<void>;
  checkAuth: () => Promise<void>;
  resendVerification: (email: string) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Helper function to get API URL based on environment
function getApiBaseUrl(siteConfig: any): string {
  // Check if we're running in a browser
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    // Use production URL for GitHub Pages or any non-localhost domain
    if (hostname !== 'localhost' && hostname !== '127.0.0.1') {
      return (siteConfig?.customFields?.apiUrlProd as string) || 'https://jiwaniz-physical-ai-backend.hf.space';
    }
  }
  // Use development URL for localhost
  return (siteConfig?.customFields?.apiUrlDev as string) || 'http://localhost:8001';
}

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = getApiBaseUrl(siteConfig);

  const [currentUser, setCurrentUser] = useState<UserResponse | null>(null);
  const [accessToken, setAccessToken] = useState<string | null>(() => {
    // Initialize from localStorage if available
    if (typeof window !== 'undefined') {
      return localStorage.getItem('access_token');
    }
    return null;
  });
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  const signup = async (email: string, password: string, name: string): Promise<void> => {
    setError(null);
    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password, name }),
        credentials: 'include', // Important for cookies
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Signup failed');
      }

      const data = await response.json();
      setCurrentUser(data.user);
      // Store token in state and localStorage
      if (data.access_token) {
        setAccessToken(data.access_token);
        localStorage.setItem('access_token', data.access_token);
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Signup failed';
      setError(message);
      throw err;
    }
  };

  const signin = async (email: string, password: string): Promise<void> => {
    setError(null);
    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
        credentials: 'include',
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Signin failed');
      }

      const data = await response.json();
      setCurrentUser(data.user);
      // Store token in state and localStorage
      if (data.access_token) {
        setAccessToken(data.access_token);
        localStorage.setItem('access_token', data.access_token);
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Signin failed';
      setError(message);
      throw err;
    }
  };

  const signout = async (): Promise<void> => {
    setError(null);
    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/signout`, {
        method: 'POST',
        credentials: 'include',
      });

      if (!response.ok) {
        throw new Error('Signout failed');
      }

      setCurrentUser(null);
      setAccessToken(null);
      localStorage.removeItem('access_token');
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Signout failed';
      setError(message);
      throw err;
    }
  };

  const checkAuth = async (): Promise<void> => {
    setIsLoading(true);
    setError(null);
    try {
      const storedToken = localStorage.getItem('access_token');
      const headers: HeadersInit = {};
      if (storedToken) {
        headers['Authorization'] = `Bearer ${storedToken}`;
      }

      const response = await fetch(`${API_BASE_URL}/api/auth/me`, {
        credentials: 'include',
        headers,
      });

      if (response.ok) {
        const data = await response.json();
        setCurrentUser(data);
      } else {
        setCurrentUser(null);
        setAccessToken(null);
        localStorage.removeItem('access_token');
      }
    } catch (err) {
      setCurrentUser(null);
      // Don't set error for checkAuth - it's expected to fail when not authenticated
    } finally {
      setIsLoading(false);
    }
  };

  const resendVerification = async (email: string): Promise<void> => {
    setError(null);
    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/resend-verification`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Failed to resend verification email');
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Failed to resend verification email';
      setError(message);
      throw err;
    }
  };

  useEffect(() => {
    checkAuth();
  }, []);

  const value = {
    currentUser,
    accessToken,
    apiBaseUrl: API_BASE_URL,
    isLoading,
    error,
    signup,
    signin,
    signout,
    checkAuth,
    resendVerification,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
