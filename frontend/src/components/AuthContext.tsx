import React, { createContext, useContext, useState, useEffect } from 'react';

interface UserResponse {
  id: number;
  email: string;
  name: string;
  is_active: boolean;
  created_at: string;
}

interface AuthContextType {
  currentUser: UserResponse | null;
  isLoading: boolean;
  error: string | null;
  signup: (email: string, password: string, name: string) => Promise<void>;
  signin: (email: string, password: string) => Promise<void>;
  signout: () => Promise<void>;
  checkAuth: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

// API base URL - adjust based on environment
// Use typeof check for Docusaurus compatibility
const API_BASE_URL = (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL) || 'http://localhost:8000';

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [currentUser, setCurrentUser] = useState<UserResponse | null>(null);
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
      const response = await fetch(`${API_BASE_URL}/api/auth/me`, {
        credentials: 'include',
      });

      if (response.ok) {
        const data = await response.json();
        setCurrentUser(data);
      } else {
        setCurrentUser(null);
      }
    } catch (err) {
      setCurrentUser(null);
      // Don't set error for checkAuth - it's expected to fail when not authenticated
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    checkAuth();
  }, []);

  const value = {
    currentUser,
    isLoading,
    error,
    signup,
    signin,
    signout,
    checkAuth,
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
