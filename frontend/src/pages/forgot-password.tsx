import React, { useState } from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from '../components/AuthContext';

export default function ForgotPassword(): JSX.Element {
  const [email, setEmail] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [sent, setSent] = useState(false);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const { resetPassword } = useAuth();
  const signinUrl = useBaseUrl('/signin');

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsSubmitting(true);

    try {
      await resetPassword(email);
      setSent(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to send reset email');
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Layout title="Forgot Password" description="Reset your password">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            {sent ? (
              <div style={{ textAlign: 'center' }}>
                <div
                  style={{
                    width: 64,
                    height: 64,
                    borderRadius: '50%',
                    background: 'linear-gradient(135deg, #4CAF50, #45a049)',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    margin: '0 auto 1.5rem',
                    fontSize: '2rem',
                    color: '#fff',
                  }}
                >
                  &#9993;
                </div>
                <h2>Check Your Email</h2>
                <p>
                  We sent a password reset link to <strong>{email}</strong>.
                </p>
                <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                  Click the link in the email to set a new password. The link expires in 1 hour.
                </p>
                <a href={signinUrl} className="button button--secondary" style={{ marginTop: '1rem' }}>
                  Back to Sign In
                </a>
              </div>
            ) : (
              <div>
                <h2>Reset Your Password</h2>
                <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                  Enter your email address and we'll send you a link to reset your password.
                </p>

                <form onSubmit={handleSubmit}>
                  {error && (
                    <div className="alert alert-danger" role="alert">
                      {error}
                    </div>
                  )}

                  <div className="form-group">
                    <label htmlFor="email">Email Address</label>
                    <input
                      type="email"
                      id="email"
                      className="form-control"
                      value={email}
                      onChange={(e) => setEmail(e.target.value)}
                      required
                      placeholder="your.email@example.com"
                    />
                  </div>

                  <button
                    type="submit"
                    className="button button--primary button--lg"
                    disabled={isSubmitting}
                  >
                    {isSubmitting ? 'Sending...' : 'Send Reset Link'}
                  </button>

                  <p className="mt-3">
                    Remember your password? <a href={signinUrl}>Sign in here</a>
                  </p>
                </form>
              </div>
            )}
          </div>
        </div>
      </main>
    </Layout>
  );
}
