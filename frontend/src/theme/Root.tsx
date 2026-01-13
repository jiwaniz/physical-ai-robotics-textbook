import React from 'react';
import { AuthProvider } from '../components/AuthContext';

// This file wraps the entire app with AuthProvider for global auth state
export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}
