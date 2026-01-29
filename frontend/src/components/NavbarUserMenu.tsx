import React from 'react';
import { useAuth } from './AuthContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

export default function NavbarUserMenu(): JSX.Element {
  const { currentUser, isLoading, signout } = useAuth();
  const signinUrl = useBaseUrl('/signin');
  const signupUrl = useBaseUrl('/signup');
  const homeUrl = useBaseUrl('/');

  const handleSignout = async () => {
    try {
      await signout();
      // Redirect to home after signout
      window.location.href = homeUrl;
    } catch (err) {
      console.error('Signout failed:', err);
    }
  };

  if (isLoading) {
    return <span className="navbar__item">Loading...</span>;
  }

  if (currentUser) {
    // Supabase stores user name in user_metadata from signup
    const displayName = currentUser.user_metadata?.name || currentUser.email?.split('@')[0] || 'User';

    return (
      <div className="navbar__item dropdown dropdown--hoverable dropdown--right">
        <span className="navbar__link" style={{ cursor: 'pointer' }}>
          {displayName}
        </span>
        <ul className="dropdown__menu">
          <li>
            <button
              className="dropdown__link"
              onClick={handleSignout}
              style={{
                background: 'none',
                border: 'none',
                width: '100%',
                textAlign: 'left',
                cursor: 'pointer',
                padding: '0.5rem 1rem',
              }}
            >
              Sign Out
            </button>
          </li>
        </ul>
      </div>
    );
  }

  return (
    <div className="navbar__item">
      <a href={signinUrl} className="navbar__link">
        Sign In
      </a>
      {' / '}
      <a href={signupUrl} className="navbar__link">
        Sign Up
      </a>
    </div>
  );
}
