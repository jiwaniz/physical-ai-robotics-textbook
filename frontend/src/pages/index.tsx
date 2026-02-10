import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Translate, { translate } from '@docusaurus/Translate';
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
        <h1 className="hero__title">
          <Translate id="page.home.heroTitle">Physical AI &amp; Humanoid Robotics</Translate>
        </h1>
        <p className="hero__subtitle">
          <Translate id="page.home.heroTagline">Interactive textbook with AI-powered learning assistance</Translate>
        </p>
        <div className="margin-top--lg">
          {isLoading ? (
            <div><Translate id="page.home.loading">Loading...</Translate></div>
          ) : currentUser ? (
            <div>
              <p><Translate id="page.home.welcomeBack" values={{ name: currentUser.user_metadata?.name || currentUser.email?.split('@')[0] || 'User' }}>{'Welcome back, {name}! 👋'}</Translate></p>
              <div className="button-group">
                <Link
                  className="button button--secondary button--lg margin-right--md"
                  to={docsUrl}>
                  <Translate id="page.home.continueLearning">Continue Learning 📚</Translate>
                </Link>
                <button
                  className="button button--outline button--secondary button--lg"
                  onClick={handleSignout}>
                  <Translate id="page.home.signOut">Sign Out</Translate>
                </button>
              </div>
            </div>
          ) : (
            <div className="button-group">
              <Link
                className="button button--secondary button--lg margin-right--md"
                to={signupUrl}>
                <Translate id="page.home.getStarted">Get Started 🚀</Translate>
              </Link>
              <Link
                className="button button--outline button--secondary button--lg"
                to={signinUrl}>
                <Translate id="page.home.signIn">Sign In</Translate>
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
              <h3><Translate id="page.home.feature.physicalAI.title">🤖 Physical AI</Translate></h3>
              <p>
                <Translate id="page.home.feature.physicalAI.description">
                  Learn to build intelligent robotic systems that interact with the physical world.
                </Translate>
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3><Translate id="page.home.feature.humanoidRobotics.title">🦾 Humanoid Robotics</Translate></h3>
              <p>
                <Translate id="page.home.feature.humanoidRobotics.description">
                  Master the fundamentals of humanoid robot design, control, and programming.
                </Translate>
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3><Translate id="page.home.feature.curriculum.title">📖 Comprehensive Curriculum</Translate></h3>
              <p>
                <Translate id="page.home.feature.curriculum.description">
                  From ROS2 basics to advanced simulation with Isaac Sim and Gazebo.
                </Translate>
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
      title={translate({ message: `Welcome to ${siteConfig.title}`, id: 'page.home.layoutTitle' })}
      description={translate({ message: 'Learn Physical AI and Humanoid Robotics from fundamentals to advanced topics', id: 'page.home.layoutDescription' })}>
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
