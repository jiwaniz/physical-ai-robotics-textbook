import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import OnboardingForm from '../components/OnboardingForm';
import { useAuth } from '../components/AuthContext';

export default function Onboarding(): JSX.Element {
  const { currentUser, isLoading } = useAuth();
  const history = useHistory();

  useEffect(() => {
    // Redirect to signin if not authenticated
    if (!isLoading && !currentUser) {
      history.push('/signin');
    }
  }, [currentUser, isLoading, history]);

  if (isLoading) {
    return (
      <Layout title="Loading...">
        <main className="container margin-vert--lg">
          <div className="text--center">
            <p>Loading...</p>
          </div>
        </main>
      </Layout>
    );
  }

  if (!currentUser) {
    return null; // Will redirect via useEffect
  }

  return (
    <Layout
      title="Complete Your Profile"
      description="Tell us about your background to personalize your learning experience"
    >
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <OnboardingForm allowSkip={true} />
          </div>
        </div>
      </main>
    </Layout>
  );
}
