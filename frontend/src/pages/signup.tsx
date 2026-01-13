import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/SignupForm';

export default function Signup(): JSX.Element {
  return (
    <Layout
      title="Sign Up"
      description="Create your account to access the Physical AI & Humanoid Robotics Textbook"
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
