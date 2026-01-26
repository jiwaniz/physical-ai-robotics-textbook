import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/AuthContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

export default function VerifyEmailPendingPage(): JSX.Element {
  const { currentUser, resendVerification } = useAuth();
  const [resendStatus, setResendStatus] = useState<'idle' | 'sending' | 'sent' | 'error'>('idle');
  const [resendMessage, setResendMessage] = useState('');
  const signinUrl = useBaseUrl('/signin');

  const handleResend = async () => {
    if (!currentUser?.email) return;

    setResendStatus('sending');
    try {
      await resendVerification(currentUser.email);
      setResendStatus('sent');
      setResendMessage('Verification email sent! Please check your inbox.');
    } catch (err) {
      setResendStatus('error');
      setResendMessage('Failed to resend. Please try again later.');
    }
  };

  return (
    <Layout title="Verify Your Email">
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
              {/* Email Icon */}
              <div
                style={{
                  width: '80px',
                  height: '80px',
                  background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                  borderRadius: '50%',
                  margin: '0 auto 1.5rem',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                }}
              >
                <svg
                  width="40"
                  height="40"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="white"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                >
                  <path d="M4 4h16c1.1 0 2 .9 2 2v12c0 1.1-.9 2-2 2H4c-1.1 0-2-.9-2-2V6c0-1.1.9-2 2-2z" />
                  <polyline points="22,6 12,13 2,6" />
                </svg>
              </div>

              <h1 style={{ marginBottom: '1rem' }}>Check Your Email</h1>

              <p style={{ color: 'var(--ifm-color-emphasis-700)', marginBottom: '1rem', fontSize: '1.1rem' }}>
                We've sent a verification link to:
              </p>

              <p style={{ fontWeight: 'bold', fontSize: '1.2rem', marginBottom: '2rem' }}>
                {currentUser?.email || currentUser?.user_metadata?.email || 'your email address'}
              </p>

              <p style={{ color: 'var(--ifm-color-emphasis-600)', marginBottom: '2rem' }}>
                Click the link in the email to verify your account and access the course.
                The link will expire in 24 hours.
              </p>

              <div style={{ borderTop: '1px solid var(--ifm-color-emphasis-300)', paddingTop: '1.5rem', marginTop: '1.5rem' }}>
                <p style={{ color: 'var(--ifm-color-emphasis-600)', marginBottom: '1rem' }}>
                  Didn't receive the email?
                </p>

                {resendStatus === 'sent' && (
                  <div style={{ color: 'var(--ifm-color-success)', marginBottom: '1rem' }}>
                    {resendMessage}
                  </div>
                )}

                {resendStatus === 'error' && (
                  <div style={{ color: 'var(--ifm-color-danger)', marginBottom: '1rem' }}>
                    {resendMessage}
                  </div>
                )}

                <button
                  onClick={handleResend}
                  className="button button--secondary button--lg"
                  disabled={resendStatus === 'sending' || resendStatus === 'sent'}
                  style={{ marginRight: '1rem' }}
                >
                  {resendStatus === 'sending' ? 'Sending...' : 'Resend Email'}
                </button>

                <a href={signinUrl} className="button button--outline button--primary button--lg">
                  Back to Sign In
                </a>
              </div>

              <p style={{ color: 'var(--ifm-color-emphasis-500)', fontSize: '0.9rem', marginTop: '2rem' }}>
                Check your spam folder if you don't see the email in your inbox.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
