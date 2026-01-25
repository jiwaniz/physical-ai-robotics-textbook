import React from 'react';
import { AuthProvider } from '../components/AuthContext';
import EmailVerificationBanner from '../components/EmailVerificationBanner';

// This file wraps the entire app with AuthProvider for global auth state
export default function Root({ children }) {
  return (
    <AuthProvider>
      <EmailVerificationBanner />
      {children}
    </AuthProvider>
  );
}
