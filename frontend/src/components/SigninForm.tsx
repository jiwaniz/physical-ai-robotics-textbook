import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import { useHistory, useLocation } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';

const SigninForm: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const { signin, signout } = useAuth();
  const history = useHistory();
  const location = useLocation();
  const baseUrl = useBaseUrl('/');
  const verifyPendingUrl = useBaseUrl('/verify-email-pending');

  // Get redirect URL from query params
  const params = new URLSearchParams(location.search);
  const redirectUrl = params.get('redirect');

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsSubmitting(true);

    try {
      const user = await signin(email, password);

      // Check if email is verified
      if (!user.email_confirmed_at) {
        // Sign out the unverified user and redirect to verification page
        await signout();
        history.push(verifyPendingUrl);
        return;
      }

      // Redirect to original page or home after successful signin
      const destination = redirectUrl ? decodeURIComponent(redirectUrl) : baseUrl;
      history.push(destination);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Signin failed');
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className="signin-form-container">
      <h2>Sign In to Your Account</h2>
      <form onSubmit={handleSubmit} className="signin-form">
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

        <div className="form-group">
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            className="form-control"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            placeholder="Enter your password"
          />
        </div>

        <div style={{ textAlign: 'right', marginBottom: '1rem' }}>
          <a href={useBaseUrl('/forgot-password')} style={{ fontSize: '0.9rem' }}>
            Forgot password?
          </a>
        </div>

        <button
          type="submit"
          className="button button--primary button--lg"
          disabled={isSubmitting}
        >
          {isSubmitting ? 'Signing In...' : 'Sign In'}
        </button>

        <p className="mt-3">
          Don't have an account? <a href={useBaseUrl('/signup')}>Sign up here</a>
        </p>
      </form>
    </div>
  );
};

export default SigninForm;
