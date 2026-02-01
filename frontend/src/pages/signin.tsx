import React from 'react';
import Layout from '@theme/Layout';
import { translate } from '@docusaurus/Translate';
import SigninForm from '../components/SigninForm';

export default function Signin(): JSX.Element {
  return (
    <Layout
      title={translate({ message: 'Sign In', id: 'page.signin.title' })}
      description={translate({ message: 'Sign in to your account to access personalized learning', id: 'page.signin.description' })}
    >
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <SigninForm />
          </div>
        </div>
      </main>
    </Layout>
  );
}
