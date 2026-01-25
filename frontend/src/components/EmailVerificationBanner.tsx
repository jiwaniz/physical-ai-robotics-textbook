import React, { useState } from 'react';
import { useAuth } from './AuthContext';

export default function EmailVerificationBanner(): JSX.Element | null {
  const { currentUser, resendVerification } = useAuth();
  const [resending, setResending] = useState(false);
  const [resent, setResent] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Only show if user is logged in but email not verified
  if (!currentUser || currentUser.email_verified) {
    return null;
  }

  const handleResend = async () => {
    setResending(true);
    setError(null);
    try {
      await resendVerification(currentUser.email);
      setResent(true);
    } catch (err) {
      setError('Failed to resend verification email. Please try again.');
    } finally {
      setResending(false);
    }
  };

  return (
    <div
      style={{
        background: 'var(--ifm-color-warning-lightest)',
        borderBottom: '1px solid var(--ifm-color-warning)',
        padding: '0.75rem 1rem',
        textAlign: 'center',
      }}
    >
      <div
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          gap: '1rem',
          flexWrap: 'wrap',
        }}
      >
        <span style={{ color: 'var(--ifm-color-warning-darker)' }}>
          <strong>Please verify your email address.</strong> Check your inbox for a verification link.
        </span>

        {resent ? (
          <span style={{ color: 'var(--ifm-color-success-darker)' }}>
            Verification email sent!
          </span>
        ) : (
          <button
            onClick={handleResend}
            disabled={resending}
            style={{
              background: 'var(--ifm-color-warning)',
              color: 'white',
              border: 'none',
              padding: '0.375rem 0.75rem',
              borderRadius: '4px',
              cursor: resending ? 'not-allowed' : 'pointer',
              fontSize: '0.875rem',
              opacity: resending ? 0.7 : 1,
            }}
          >
            {resending ? 'Sending...' : 'Resend Email'}
          </button>
        )}

        {error && (
          <span style={{ color: 'var(--ifm-color-danger)' }}>{error}</span>
        )}
      </div>
    </div>
  );
}
