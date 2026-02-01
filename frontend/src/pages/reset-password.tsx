import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import Translate, { translate } from '@docusaurus/Translate';
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
      setError(translate({ message: 'Password must be at least 8 characters', id: 'page.resetPassword.error.tooShort' }));
      return;
    }

    if (password !== confirmPassword) {
      setError(translate({ message: 'Passwords do not match', id: 'page.resetPassword.error.mismatch' }));
      return;
    }

    setIsSubmitting(true);

    try {
      await updatePassword(password);
      setSuccess(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : translate({ message: 'Failed to update password', id: 'page.resetPassword.error.updateFailed' }));
    } finally {
      setIsSubmitting(false);
    }
  };

  if (!ready) {
    return (
      <Layout title={translate({ message: 'Reset Password', id: 'page.resetPassword.title' })}>
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
            <p><Translate id="page.resetPassword.processing">Processing reset link...</Translate></p>
          </div>
        </main>
      </Layout>
    );
  }

  if (!currentUser) {
    return (
      <Layout title={translate({ message: 'Reset Password', id: 'page.resetPassword.title' })}>
        <main className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3" style={{ textAlign: 'center' }}>
              <h2><Translate id="page.resetPassword.invalidLink.heading">Invalid or Expired Link</Translate></h2>
              <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                <Translate id="page.resetPassword.invalidLink.message">This password reset link is invalid or has expired. Please request a new one.</Translate>
              </p>
              <a href={useBaseUrl('/forgot-password')} className="button button--primary">
                <Translate id="page.resetPassword.requestNewLink">Request New Link</Translate>
              </a>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  if (success) {
    return (
      <Layout title={translate({ message: 'Password Updated', id: 'page.resetPassword.updatedTitle' })}>
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
              <h2><Translate id="page.resetPassword.success.heading">Password Updated</Translate></h2>
              <p><Translate id="page.resetPassword.success.message">Your password has been successfully changed. You can now sign in with your new password.</Translate></p>
              <a href={signinUrl} className="button button--primary" style={{ marginTop: '1rem' }}>
                <Translate id="page.resetPassword.signIn">Sign In</Translate>
              </a>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title={translate({ message: 'Set New Password', id: 'page.resetPassword.setNewTitle' })} description={translate({ message: 'Set your new password', id: 'page.resetPassword.setNewDescription' })}>
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h2><Translate id="page.resetPassword.form.heading">Set New Password</Translate></h2>
            <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
              <Translate id="page.resetPassword.form.instructions">Enter your new password below.</Translate>
            </p>

            <form onSubmit={handleSubmit}>
              {error && (
                <div className="alert alert-danger" role="alert">
                  {error}
                </div>
              )}

              <div className="form-group">
                <label htmlFor="password"><Translate id="page.resetPassword.newPasswordLabel">New Password</Translate></label>
                <input
                  type="password"
                  id="password"
                  className="form-control"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                  minLength={8}
                  placeholder={translate({ message: 'At least 8 characters', id: 'page.resetPassword.newPasswordPlaceholder' })}
                />
              </div>

              <div className="form-group">
                <label htmlFor="confirmPassword"><Translate id="page.resetPassword.confirmPasswordLabel">Confirm New Password</Translate></label>
                <input
                  type="password"
                  id="confirmPassword"
                  className="form-control"
                  value={confirmPassword}
                  onChange={(e) => setConfirmPassword(e.target.value)}
                  required
                  minLength={8}
                  placeholder={translate({ message: 'Repeat your new password', id: 'page.resetPassword.confirmPasswordPlaceholder' })}
                />
              </div>

              <button
                type="submit"
                className="button button--primary button--lg"
                disabled={isSubmitting}
              >
                {isSubmitting ? translate({ message: 'Updating...', id: 'page.resetPassword.updating' }) : translate({ message: 'Update Password', id: 'page.resetPassword.submit' })}
              </button>
            </form>
          </div>
        </div>
      </main>
    </Layout>
  );
}
