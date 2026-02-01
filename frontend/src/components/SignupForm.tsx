import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import { useHistory } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Translate, { translate } from '@docusaurus/Translate';

const SignupForm: React.FC = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const { signup } = useAuth();
  const history = useHistory();
  const verifyPendingUrl = useBaseUrl('/verify-email-pending');

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsSubmitting(true);

    try {
      await signup(email, password, name);
      // Redirect to email verification pending page
      history.push(verifyPendingUrl);
    } catch (err) {
      setError(err instanceof Error ? err.message : translate({ message: 'Signup failed', id: 'component.signupForm.errorFallback' }));
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2><Translate id="component.signupForm.heading">Create Your Account</Translate></h2>
      <form onSubmit={handleSubmit} className="signup-form">
        {error && (
          <div className="alert alert-danger" role="alert">
            {error}
          </div>
        )}

        <div className="form-group">
          <label htmlFor="name"><Translate id="component.signupForm.nameLabel">Full Name</Translate></label>
          <input
            type="text"
            id="name"
            className="form-control"
            value={name}
            onChange={(e) => setName(e.target.value)}
            required
            placeholder={translate({ message: 'Enter your full name', id: 'component.signupForm.namePlaceholder' })}
          />
        </div>

        <div className="form-group">
          <label htmlFor="email"><Translate id="component.signupForm.emailLabel">Email Address</Translate></label>
          <input
            type="email"
            id="email"
            className="form-control"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            placeholder={translate({ message: 'your.email@example.com', id: 'component.signupForm.emailPlaceholder' })}
          />
        </div>

        <div className="form-group">
          <label htmlFor="password"><Translate id="component.signupForm.passwordLabel">Password</Translate></label>
          <input
            type="password"
            id="password"
            className="form-control"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            minLength={8}
            placeholder={translate({ message: 'At least 8 characters', id: 'component.signupForm.passwordPlaceholder' })}
          />
          <small className="form-text text-muted">
            <Translate id="component.signupForm.passwordHint">Password must be at least 8 characters and contain uppercase, lowercase, and numbers.</Translate>
          </small>
        </div>

        <button
          type="submit"
          className="button button--primary button--lg"
          disabled={isSubmitting}
        >
          {isSubmitting ? <Translate id="component.signupForm.submitting">Creating Account...</Translate> : <Translate id="component.signupForm.submit">Sign Up</Translate>}
        </button>

        <p className="mt-3">
          <Translate id="component.signupForm.hasAccount">Already have an account?</Translate>{' '}<a href={useBaseUrl('/signin')}><Translate id="component.signupForm.signinLink">Sign in here</Translate></a>
        </p>
      </form>
    </div>
  );
};

export default SignupForm;
