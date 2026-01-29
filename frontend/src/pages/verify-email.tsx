import React, { useEffect, useState, useRef } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from '../components/AuthContext';

export default function VerifyEmailPage(): JSX.Element {
  const { currentUser, isLoading, refreshSession } = useAuth();
  const history = useHistory();
  const onboardingUrl = useBaseUrl('/onboarding');
  const signinUrl = useBaseUrl('/signin');
  const [status, setStatus] = useState<'loading' | 'success' | 'error'>('loading');
  const [message, setMessage] = useState('Verifying your email...');
  const hasChecked = useRef(false);
  const checkCount = useRef(0);

  // Check if URL has auth tokens (Supabase puts them in the hash)
  const hasAuthTokensInUrl = () => {
    if (typeof window === 'undefined') return false;
    const hash = window.location.hash;
    return hash.includes('access_token') || hash.includes('type=signup') || hash.includes('type=recovery');
  };

  useEffect(() => {
    if (isLoading) return;

    // If we already determined the final status, don't re-check
    if (hasChecked.current && status !== 'loading') return;

    const checkVerification = () => {
      checkCount.current += 1;

      // If user is verified, success!
      if (currentUser?.email_confirmed_at) {
        hasChecked.current = true;
        setStatus('success');
        setMessage('Your email has been verified successfully! Redirecting to onboarding...');
        setTimeout(() => {
          history.push(onboardingUrl);
        }, 2000);
        return;
      }

      // If user exists but not verified
      if (currentUser && !currentUser.email_confirmed_at) {
        hasChecked.current = true;
        setStatus('error');
        setMessage('Email verification is still pending. Please click the link in your email.');
        return;
      }

      // No user yet - might still be processing URL hash tokens
      // Wait up to 5 seconds (5 checks) for Supabase to process
      if (!currentUser && checkCount.current < 5) {
        setTimeout(checkVerification, 1000);
        return;
      }

      // After waiting, still no user
      hasChecked.current = true;
      setStatus('error');
      setMessage('Please sign in to verify your email.');
    };

    // Initial delay to let Supabase process URL hash
    const initialDelay = hasAuthTokensInUrl() ? 2000 : 500;
    setTimeout(checkVerification, initialDelay);
  }, [isLoading, currentUser, history, onboardingUrl, status]);

  return (
    <Layout title="Email Verification">
      <main className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div
              style={{
                textAlign: 'center',
                padding: '3rem 2rem',
                background: 'var(--ifm-background-surface-color)',
                borderRadius: '12px',
                boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
              }}
            >
              {status === 'loading' && (
                <>
                  <div
                    style={{
                      width: '48px',
                      height: '48px',
                      border: '4px solid #e0e0e0',
                      borderTopColor: 'var(--ifm-color-primary)',
                      borderRadius: '50%',
                      margin: '0 auto 1.5rem',
                      animation: 'spin 1s linear infinite',
                    }}
                  />
                  <style>
                    {`@keyframes spin { to { transform: rotate(360deg); } }`}
                  </style>
                </>
              )}

              {status === 'success' && (
                <div
                  style={{
                    width: '64px',
                    height: '64px',
                    background: 'var(--ifm-color-success)',
                    borderRadius: '50%',
                    margin: '0 auto 1.5rem',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                  }}
                >
                  <svg
                    width="32"
                    height="32"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="white"
                    strokeWidth="3"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  >
                    <polyline points="20 6 9 17 4 12" />
                  </svg>
                </div>
              )}

              {status === 'error' && (
                <div
                  style={{
                    width: '64px',
                    height: '64px',
                    background: 'var(--ifm-color-danger)',
                    borderRadius: '50%',
                    margin: '0 auto 1.5rem',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                  }}
                >
                  <svg
                    width="32"
                    height="32"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="white"
                    strokeWidth="3"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  >
                    <line x1="18" y1="6" x2="6" y2="18" />
                    <line x1="6" y1="6" x2="18" y2="18" />
                  </svg>
                </div>
              )}

              <h2 style={{ marginBottom: '1rem' }}>
                {status === 'loading' && 'Verifying Email'}
                {status === 'success' && 'Email Verified!'}
                {status === 'error' && 'Verification Issue'}
              </h2>

              <p style={{ color: 'var(--ifm-color-emphasis-700)', marginBottom: '2rem' }}>
                {message}
              </p>

              {status === 'success' && (
                <button
                  onClick={() => history.push(onboardingUrl)}
                  className="button button--primary button--lg"
                >
                  Continue to Onboarding Now
                </button>
              )}

              {status === 'error' && (
                <div style={{ display: 'flex', gap: '1rem', justifyContent: 'center' }}>
                  <button
                    onClick={() => history.push(signinUrl)}
                    className="button button--secondary button--lg"
                  >
                    Sign In
                  </button>
                  <button
                    onClick={() => history.push('/')}
                    className="button button--primary button--lg"
                  >
                    Go Home
                  </button>
                </div>
              )}
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
