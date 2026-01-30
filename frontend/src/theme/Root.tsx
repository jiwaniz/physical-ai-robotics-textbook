import React from 'react';
import { AuthProvider } from '../components/AuthContext';
import EmailVerificationBanner from '../components/EmailVerificationBanner';
import ChatWidget from '../components/ChatWidget';

// This file wraps the entire app with providers for global state
export default function Root({ children }) {
  return (
    <AuthProvider>
      <EmailVerificationBanner />
      {children}
      <ChatWidget />
    </AuthProvider>
  );
}
