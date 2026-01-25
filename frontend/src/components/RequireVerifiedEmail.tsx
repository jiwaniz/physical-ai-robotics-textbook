import React from 'react';
import { Redirect } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from './AuthContext';

interface RequireVerifiedEmailProps {
  children: React.ReactNode;
}

/**
 * Wrapper component that requires email verification before showing content.
 * Redirects to:
 * - Sign in page if not authenticated
 * - Verification pending page if authenticated but email not verified
 */
export default function RequireVerifiedEmail({ children }: RequireVerifiedEmailProps): JSX.Element {
  const { currentUser, isLoading } = useAuth();
  const signinUrl = useBaseUrl('/signin');
  const verifyPendingUrl = useBaseUrl('/verify-email-pending');

  // Show loading state while checking auth
  if (isLoading) {
    return (
      <div style={{ display: 'flex', justifyContent: 'center', padding: '4rem' }}>
        <div
          style={{
            width: '48px',
            height: '48px',
            border: '4px solid #e0e0e0',
            borderTopColor: 'var(--ifm-color-primary)',
            borderRadius: '50%',
            animation: 'spin 1s linear infinite',
          }}
        />
        <style>
          {`@keyframes spin { to { transform: rotate(360deg); } }`}
        </style>
      </div>
    );
  }

  // Redirect to sign in if not authenticated
  if (!currentUser) {
    return <Redirect to={signinUrl} />;
  }

  // Redirect to verification pending if email not verified
  if (!currentUser.email_verified) {
    return <Redirect to={verifyPendingUrl} />;
  }

  // User is authenticated and verified - show content
  return <>{children}</>;
}
