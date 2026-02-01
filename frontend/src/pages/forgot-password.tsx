import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Translate, { translate } from '@docusaurus/Translate';
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
      setError(err instanceof Error ? err.message : translate({ message: 'Failed to send reset email', id: 'page.forgotPassword.error.sendFailed' }));
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Layout title={translate({ message: 'Forgot Password', id: 'page.forgotPassword.title' })} description={translate({ message: 'Reset your password', id: 'page.forgotPassword.description' })}>
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
                <h2><Translate id="page.forgotPassword.sent.heading">Check Your Email</Translate></h2>
                <p>
                  <Translate id="page.forgotPassword.sent.message" values={{ email: <strong>{email}</strong> }}>
                    {'We sent a password reset link to {email}.'}
                  </Translate>
                </p>
                <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                  <Translate id="page.forgotPassword.sent.instructions">Click the link in the email to set a new password. The link expires in 1 hour.</Translate>
                </p>
                <a href={signinUrl} className="button button--secondary" style={{ marginTop: '1rem' }}>
                  <Translate id="page.forgotPassword.backToSignIn">Back to Sign In</Translate>
                </a>
              </div>
            ) : (
              <div>
                <h2><Translate id="page.forgotPassword.heading">Reset Your Password</Translate></h2>
                <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                  <Translate id="page.forgotPassword.instructions">Enter your email address and we'll send you a link to reset your password.</Translate>
                </p>

                <form onSubmit={handleSubmit}>
                  {error && (
                    <div className="alert alert-danger" role="alert">
                      {error}
                    </div>
                  )}

                  <div className="form-group">
                    <label htmlFor="email"><Translate id="page.forgotPassword.emailLabel">Email Address</Translate></label>
                    <input
                      type="email"
                      id="email"
                      className="form-control"
                      value={email}
                      onChange={(e) => setEmail(e.target.value)}
                      required
                      placeholder={translate({ message: 'your.email@example.com', id: 'page.forgotPassword.emailPlaceholder' })}
                    />
                  </div>

                  <button
                    type="submit"
                    className="button button--primary button--lg"
                    disabled={isSubmitting}
                  >
                    {isSubmitting ? translate({ message: 'Sending...', id: 'page.forgotPassword.sending' }) : translate({ message: 'Send Reset Link', id: 'page.forgotPassword.submit' })}
                  </button>

                  <p className="mt-3">
                    <Translate id="page.forgotPassword.rememberPassword" values={{ signInLink: <a href={signinUrl}>{translate({ message: 'Sign in here', id: 'page.forgotPassword.signInLink' })}</a> }}>
                      {'Remember your password? {signInLink}'}
                    </Translate>
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
