import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from '../components/AuthContext';

export default function ResetPassword(): JSX.Element {
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [ready, setReady] = useState(false);
  const { updatePassword, isLoading, currentUser } = useAuth();
  const signinUrl = useBaseUrl('/signin');

  // Wait for Supabase to process the recovery token from the URL hash
  useEffect(() => {
    if (!isLoading) {
      // Give Supabase a moment to process the hash tokens
      const timer = setTimeout(() => setReady(true), 1000);
      return () => clearTimeout(timer);
    }
  }, [isLoading]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    setIsSubmitting(true);

    try {
      await updatePassword(password);
      setSuccess(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to update password');
    } finally {
      setIsSubmitting(false);
    }
  };

  if (!ready) {
    return (
      <Layout title="Reset Password">
        <main className="container margin-vert--lg">
          <div className="text--center">
            <div
              style={{
                width: 48,
                height: 48,
                border: '4px solid #e0e0e0',
                borderTopColor: 'var(--ifm-color-primary)',
                borderRadius: '50%',
                margin: '0 auto 1rem',
                animation: 'spin 1s linear infinite',
              }}
            />
            <style>{`@keyframes spin { to { transform: rotate(360deg); } }`}</style>
            <p>Processing reset link...</p>
          </div>
        </main>
      </Layout>
    );
  }

  if (!currentUser) {
    return (
      <Layout title="Reset Password">
        <main className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3" style={{ textAlign: 'center' }}>
              <h2>Invalid or Expired Link</h2>
              <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                This password reset link is invalid or has expired. Please request a new one.
              </p>
              <a href={useBaseUrl('/forgot-password')} className="button button--primary">
                Request New Link
              </a>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  if (success) {
    return (
      <Layout title="Password Updated">
        <main className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3" style={{ textAlign: 'center' }}>
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
                &#10003;
              </div>
              <h2>Password Updated</h2>
              <p>Your password has been successfully changed. You can now sign in with your new password.</p>
              <a href={signinUrl} className="button button--primary" style={{ marginTop: '1rem' }}>
                Sign In
              </a>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Set New Password" description="Set your new password">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h2>Set New Password</h2>
            <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
              Enter your new password below.
            </p>

            <form onSubmit={handleSubmit}>
              {error && (
                <div className="alert alert-danger" role="alert">
                  {error}
                </div>
              )}

              <div className="form-group">
                <label htmlFor="password">New Password</label>
                <input
                  type="password"
                  id="password"
                  className="form-control"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                  minLength={8}
                  placeholder="At least 8 characters"
                />
              </div>

              <div className="form-group">
                <label htmlFor="confirmPassword">Confirm New Password</label>
                <input
                  type="password"
                  id="confirmPassword"
                  className="form-control"
                  value={confirmPassword}
                  onChange={(e) => setConfirmPassword(e.target.value)}
                  required
                  minLength={8}
                  placeholder="Repeat your new password"
                />
              </div>

              <button
                type="submit"
                className="button button--primary button--lg"
                disabled={isSubmitting}
              >
                {isSubmitting ? 'Updating...' : 'Update Password'}
              </button>
            </form>
          </div>
        </div>
      </main>
    </Layout>
  );
}
