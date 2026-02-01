import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import { useHistory, useLocation } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Translate, { translate } from '@docusaurus/Translate';

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
      setError(err instanceof Error ? err.message : translate({ message: 'Signin failed', id: 'component.signinForm.errorFallback' }));
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className="signin-form-container">
      <h2><Translate id="component.signinForm.heading">Sign In to Your Account</Translate></h2>
      <form onSubmit={handleSubmit} className="signin-form">
        {error && (
          <div className="alert alert-danger" role="alert">
            {error}
          </div>
        )}

        <div className="form-group">
          <label htmlFor="email"><Translate id="component.signinForm.emailLabel">Email Address</Translate></label>
          <input
            type="email"
            id="email"
            className="form-control"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            placeholder={translate({ message: 'your.email@example.com', id: 'component.signinForm.emailPlaceholder' })}
          />
        </div>

        <div className="form-group">
          <label htmlFor="password"><Translate id="component.signinForm.passwordLabel">Password</Translate></label>
          <input
            type="password"
            id="password"
            className="form-control"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            placeholder={translate({ message: 'Enter your password', id: 'component.signinForm.passwordPlaceholder' })}
          />
        </div>

        <div style={{ textAlign: 'right', marginBottom: '1rem' }}>
          <a href={useBaseUrl('/forgot-password')} style={{ fontSize: '0.9rem' }}>
            <Translate id="component.signinForm.forgotPassword">Forgot password?</Translate>
          </a>
        </div>

        <button
          type="submit"
          className="button button--primary button--lg"
          disabled={isSubmitting}
        >
          {isSubmitting ? <Translate id="component.signinForm.submitting">Signing In...</Translate> : <Translate id="component.signinForm.submit">Sign In</Translate>}
        </button>

        <p className="mt-3">
          <Translate id="component.signinForm.noAccount">Don't have an account?</Translate>{' '}<a href={useBaseUrl('/signup')}><Translate id="component.signinForm.signupLink">Sign up here</Translate></a>
        </p>
      </form>
    </div>
  );
};

export default SigninForm;
