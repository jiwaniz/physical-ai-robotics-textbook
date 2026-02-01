import React from 'react';
import Layout from '@theme/Layout';
import { translate } from '@docusaurus/Translate';
import OnboardingForm from '../components/OnboardingForm';
import RequireVerifiedEmail from '../components/RequireVerifiedEmail';

export default function Onboarding(): JSX.Element {
  return (
    <Layout
      title={translate({ message: 'Complete Your Profile', id: 'page.onboarding.title' })}
      description={translate({ message: 'Tell us about your background to personalize your learning experience', id: 'page.onboarding.description' })}
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
