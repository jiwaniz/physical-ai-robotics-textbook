import React from 'react';
import Layout from '@theme/Layout';
import { translate } from '@docusaurus/Translate';
import SignupForm from '../components/SignupForm';

export default function Signup(): JSX.Element {
  return (
    <Layout
      title={translate({ message: 'Sign Up', id: 'page.signup.title' })}
      description={translate({ message: 'Create your account to access the Physical AI & Humanoid Robotics Textbook', id: 'page.signup.description' })}
    >
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <SignupForm />
          </div>
        </div>
      </main>
    </Layout>
  );
}
