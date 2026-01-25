import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { useAuth } from '../components/AuthContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const { currentUser, isLoading, signout } = useAuth();
  const signupUrl = useBaseUrl('/signup');
  const signinUrl = useBaseUrl('/signin');
  const docsUrl = useBaseUrl('/intro');

  const handleSignout = async () => {
    try {
      await signout();
    } catch (err) {
      console.error('Signout failed:', err);
    }
  };

  return (
    <header className={clsx('hero hero--primary')}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className="margin-top--lg">
          {isLoading ? (
            <div>Loading...</div>
          ) : currentUser ? (
            <div>
              <p>Welcome back, {currentUser.name}! 👋</p>
              <div className="button-group">
                <Link
                  className="button button--secondary button--lg margin-right--md"
                  to={docsUrl}>
                  Continue Learning 📚
                </Link>
                <button
                  className="button button--outline button--secondary button--lg"
                  onClick={handleSignout}>
                  Sign Out
                </button>
              </div>
            </div>
          ) : (
            <div className="button-group">
              <Link
                className="button button--secondary button--lg margin-right--md"
                to={signupUrl}>
                Get Started 🚀
              </Link>
              <Link
                className="button button--outline button--secondary button--lg"
                to={signinUrl}>
                Sign In
              </Link>
            </div>
          )}
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  return (
    <section className="padding-vert--xl">
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>🤖 Physical AI</h3>
              <p>
                Learn to build intelligent robotic systems that interact with the physical world.
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>🦾 Humanoid Robotics</h3>
              <p>
                Master the fundamentals of humanoid robot design, control, and programming.
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>📖 Comprehensive Curriculum</h3>
              <p>
                From ROS2 basics to advanced simulation with Isaac Sim and Gazebo.
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn Physical AI and Humanoid Robotics from fundamentals to advanced topics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
