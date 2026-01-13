import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import { useHistory } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';

const SigninForm: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const { signin } = useAuth();
  const history = useHistory();
  const baseUrl = useBaseUrl('/');

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsSubmitting(true);

    try {
      await signin(email, password);
      // Redirect to home after successful signin
      history.push(baseUrl);
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
