import React from 'react';
import { AuthProvider } from '../components/AuthContext';
import { LanguageProvider } from '../components/LanguageContext';
import EmailVerificationBanner from '../components/EmailVerificationBanner';
import ChatWidget from '../components/ChatWidget';

// This file wraps the entire app with providers for global state
export default function Root({ children }) {
  return (
    <LanguageProvider>
      <AuthProvider>
        <EmailVerificationBanner />
        {children}
        <ChatWidget />
      </AuthProvider>
    </LanguageProvider>
  );
}
