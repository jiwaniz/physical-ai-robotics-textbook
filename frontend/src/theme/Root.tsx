import React, { useEffect } from 'react';
import { AuthProvider, useAuth } from '../components/AuthContext';
import EmailVerificationBanner from '../components/EmailVerificationBanner';
import ChatWidget from '../components/ChatWidget';

function LocaleRedirect({ children }: { children: React.ReactNode }) {
  const { preferredLanguage, isLoading } = useAuth();

  useEffect(() => {
    if (isLoading || typeof window === 'undefined') return;

    const path = window.location.pathname;
    const isUrdu = path.includes('/ur/');
    const currentLocale = isUrdu ? 'ur' : 'en';

    // Only redirect if preference differs from current locale
    // and the user didn't just click the locale dropdown (no referrer check needed —
    // Docusaurus locale switch navigates, so this only fires on fresh page loads)
    if (preferredLanguage !== currentLocale) {
      if (preferredLanguage === 'ur' && !isUrdu) {
        // Redirect to Urdu version
        const baseUrl = '/physical-ai-robotics-textbook/';
        const relativePath = path.replace(baseUrl, '');
        window.location.replace(`${baseUrl}ur/${relativePath}`);
      } else if (preferredLanguage === 'en' && isUrdu) {
        // Redirect to English version
        const newPath = path.replace('/ur/', '/');
        window.location.replace(newPath);
      }
    }
  }, [preferredLanguage, isLoading]);

  return <>{children}</>;
}

function LocaleSyncOnChange({ children }: { children: React.ReactNode }) {
  const { setPreferredLanguage } = useAuth();

  useEffect(() => {
    if (typeof window === 'undefined') return;

    const path = window.location.pathname;
    const isUrdu = path.includes('/ur/');
    const currentLocale = isUrdu ? 'ur' : 'en';
    const stored = localStorage.getItem('preferred-language') || 'en';

    // If user navigated via locale dropdown, the URL locale may differ from stored preference
    // Update stored preference to match current URL locale
    if (currentLocale !== stored) {
      setPreferredLanguage(currentLocale);
    }
  }, []);

  return <>{children}</>;
}

// This file wraps the entire app with providers for global state
export default function Root({ children }) {
  return (
    <AuthProvider>
      <EmailVerificationBanner />
      <LocaleSyncOnChange>
        <LocaleRedirect>
          {children}
        </LocaleRedirect>
      </LocaleSyncOnChange>
      <ChatWidget />
    </AuthProvider>
  );
}
