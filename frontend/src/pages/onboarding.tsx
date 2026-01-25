import React from 'react';
import Layout from '@theme/Layout';
import OnboardingForm from '../components/OnboardingForm';
import RequireVerifiedEmail from '../components/RequireVerifiedEmail';

export default function Onboarding(): JSX.Element {
  return (
    <Layout
      title="Complete Your Profile"
      description="Tell us about your background to personalize your learning experience"
    >
      <RequireVerifiedEmail>
        <main className="container margin-vert--lg">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <OnboardingForm allowSkip={true} />
            </div>
          </div>
        </main>
      </RequireVerifiedEmail>
    </Layout>
  );
}
