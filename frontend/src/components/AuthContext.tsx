import React, { createContext, useContext, useState, useEffect } from 'react';
import { createClient, SupabaseClient, User, Session } from '@supabase/supabase-js';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface AuthContextType {
  currentUser: User | null;
  session: Session | null;
  accessToken: string | null;
  isLoading: boolean;
  error: string | null;
  apiBaseUrl: string;
  preferredLanguage: string;
  setPreferredLanguage: (lang: string) => Promise<void>;
  signup: (email: string, password: string, name: string) => Promise<void>;
  signin: (email: string, password: string) => Promise<User>;
  signout: () => Promise<void>;
  resendVerification: (email: string) => Promise<void>;
  refreshSession: () => Promise<void>;
  resetPassword: (email: string) => Promise<void>;
  updatePassword: (newPassword: string) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Supabase client singleton
let supabaseClient: SupabaseClient | null = null;

function getSupabaseClient(supabaseUrl: string, supabaseAnonKey: string): SupabaseClient {
  if (!supabaseClient) {
    supabaseClient = createClient(supabaseUrl, supabaseAnonKey, {
      auth: {
        autoRefreshToken: true,
        persistSession: true,
        detectSessionInUrl: true,
      },
    });
  }
  return supabaseClient;
}

// Helper function to get API URL based on environment
function getApiBaseUrl(siteConfig: any): string {
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    if (hostname !== 'localhost' && hostname !== '127.0.0.1') {
      return (siteConfig?.customFields?.apiUrlProd as string) || 'https://jiwaniz-physical-ai-backend.hf.space';
    }
  }
  return (siteConfig?.customFields?.apiUrlDev as string) || 'http://localhost:8001';
}

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = getApiBaseUrl(siteConfig);

  const supabaseUrl = siteConfig?.customFields?.supabaseUrl as string;
  const supabaseAnonKey = siteConfig?.customFields?.supabaseAnonKey as string;

  const [currentUser, setCurrentUser] = useState<User | null>(null);
  const [session, setSession] = useState<Session | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [supabase, setSupabase] = useState<SupabaseClient | null>(null);
  const [preferredLanguage, setPreferredLanguageState] = useState<string>(() => {
    if (typeof window !== 'undefined') {
      return localStorage.getItem('preferred-language') || 'en';
    }
    return 'en';
  });

  // Initialize Supabase client
  useEffect(() => {
    if (supabaseUrl && supabaseAnonKey && typeof window !== 'undefined') {
      const client = getSupabaseClient(supabaseUrl, supabaseAnonKey);
      setSupabase(client);

      // Get initial session
      client.auth.getSession().then(async ({ data: { session } }) => {
        setSession(session);
        setCurrentUser(session?.user ?? null);
        setIsLoading(false);

        // Fetch language preference from profile if authenticated
        if (session?.access_token) {
          try {
            const res = await fetch(`${API_BASE_URL}/api/users/profile`, {
              headers: { Authorization: `Bearer ${session.access_token}` },
            });
            if (res.ok) {
              const profile = await res.json();
              if (profile.preferred_language) {
                setPreferredLanguageState(profile.preferred_language);
                localStorage.setItem('preferred-language', profile.preferred_language);
              }
            }
          } catch {
            // Fall back to localStorage value
          }
        }
      });

      // Listen for auth changes
      const { data: { subscription } } = client.auth.onAuthStateChange((_event, session) => {
        setSession(session);
        setCurrentUser(session?.user ?? null);
        setIsLoading(false);
      });

      return () => {
        subscription.unsubscribe();
      };
    } else {
      setIsLoading(false);
    }
  }, [supabaseUrl, supabaseAnonKey]);

  const signup = async (email: string, password: string, name: string): Promise<void> => {
    if (!supabase) throw new Error('Supabase not initialized');

    setError(null);
    const redirectUrl = typeof window !== 'undefined'
      ? `${window.location.origin}${siteConfig.baseUrl}verify-email`
      : undefined;

    const { data, error } = await supabase.auth.signUp({
      email,
      password,
      options: {
        data: { name },
        emailRedirectTo: redirectUrl,
      },
    });

    if (error) {
      setError(error.message);
      throw error;
    }

    // User is created but needs to verify email
    if (data.user && !data.session) {
      // Email confirmation required
      setCurrentUser(data.user);
    }
  };

  const signin = async (email: string, password: string): Promise<User> => {
    if (!supabase) throw new Error('Supabase not initialized');

    setError(null);
    const { data, error } = await supabase.auth.signInWithPassword({
      email,
      password,
    });

    if (error) {
      setError(error.message);
      throw error;
    }

    setSession(data.session);
    setCurrentUser(data.user);
    return data.user;
  };

  const signout = async (): Promise<void> => {
    if (!supabase) throw new Error('Supabase not initialized');

    setError(null);
    const { error } = await supabase.auth.signOut();

    if (error) {
      setError(error.message);
      throw error;
    }

    setSession(null);
    setCurrentUser(null);
  };

  const resendVerification = async (email: string): Promise<void> => {
    if (!supabase) throw new Error('Supabase not initialized');

    setError(null);
    const redirectUrl = typeof window !== 'undefined'
      ? `${window.location.origin}${siteConfig.baseUrl}verify-email`
      : undefined;

    const { error } = await supabase.auth.resend({
      type: 'signup',
      email,
      options: {
        emailRedirectTo: redirectUrl,
      },
    });

    if (error) {
      setError(error.message);
      throw error;
    }
  };

  const resetPassword = async (email: string): Promise<void> => {
    if (!supabase) throw new Error('Supabase not initialized');

    setError(null);
    const redirectUrl = typeof window !== 'undefined'
      ? `${window.location.origin}${siteConfig.baseUrl}reset-password`
      : undefined;

    const { error } = await supabase.auth.resetPasswordForEmail(email, {
      redirectTo: redirectUrl,
    });

    if (error) {
      setError(error.message);
      throw error;
    }
  };

  const updatePassword = async (newPassword: string): Promise<void> => {
    if (!supabase) throw new Error('Supabase not initialized');

    setError(null);
    const { error } = await supabase.auth.updateUser({ password: newPassword });

    if (error) {
      setError(error.message);
      throw error;
    }
  };

  const refreshSession = async (): Promise<void> => {
    if (!supabase) throw new Error('Supabase not initialized');

    // Refresh the session to get updated user data (including email_confirmed_at)
    const { data, error } = await supabase.auth.refreshSession();

    if (error) {
      console.error('Failed to refresh session:', error);
      return;
    }

    if (data.session) {
      setSession(data.session);
      setCurrentUser(data.user);
    }
  };

  const setPreferredLanguage = async (lang: string): Promise<void> => {
    setPreferredLanguageState(lang);
    localStorage.setItem('preferred-language', lang);

    // Persist to Supabase if authenticated
    if (session?.access_token) {
      try {
        await fetch(`${API_BASE_URL}/api/users/profile`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            Authorization: `Bearer ${session.access_token}`,
          },
          body: JSON.stringify({ preferred_language: lang }),
        });
      } catch {
        // localStorage already saved as fallback
      }
    }
  };

  const value = {
    currentUser,
    session,
    accessToken: session?.access_token ?? null,
    isLoading,
    error,
    apiBaseUrl: API_BASE_URL,
    preferredLanguage,
    setPreferredLanguage,
    signup,
    signin,
    signout,
    resendVerification,
    refreshSession,
    resetPassword,
    updatePassword,
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
