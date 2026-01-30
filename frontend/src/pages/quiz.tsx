import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory, useLocation } from '@docusaurus/router';
import { useAuth } from '../components/AuthContext';
import QuizPlayer from '../components/quiz/QuizPlayer';

interface QuizInfo {
  id: number;
  week_number: number;
  chapter: string;
  title: string;
  description: string;
  time_limit_minutes: number;
  max_attempts: number;
  passing_score: number;
  question_count: number;
  user_attempts_count?: number;
  user_best_score?: number;
}

export default function QuizPage(): JSX.Element {
  const { currentUser, isLoading: authLoading, accessToken, apiBaseUrl } = useAuth();
  const history = useHistory();
  const location = useLocation();
  const [quizInfo, setQuizInfo] = useState<QuizInfo | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [started, setStarted] = useState(false);

  // Parse query params
  const params = new URLSearchParams(location.search);
  const quizId = params.get('id');
  const weekNumber = params.get('week');

  useEffect(() => {
    if (!authLoading && !currentUser) {
      history.push('/signin?redirect=/quiz' + location.search);
    }
  }, [currentUser, authLoading, history, location.search]);

  useEffect(() => {
    const fetchQuizInfo = async () => {
      if (!accessToken) return;

      setLoading(true);
      setError(null);

      try {
        let endpoint = '';
        if (quizId) {
          endpoint = `${apiBaseUrl}/api/quizzes/${quizId}`;
        } else if (weekNumber) {
          endpoint = `${apiBaseUrl}/api/quizzes/week/${weekNumber}`;
        } else {
          setError('No quiz ID or week number provided');
          setLoading(false);
          return;
        }

        const response = await fetch(endpoint, {
          headers: {
            Authorization: `Bearer ${accessToken}`,
          },
          credentials: 'include',
        });

        if (!response.ok) {
          const data = await response.json();
          throw new Error(data.detail || 'Failed to load quiz');
        }

        const data = await response.json();
        setQuizInfo(data);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Failed to load quiz');
      } finally {
        setLoading(false);
      }
    };

    if (currentUser && accessToken) {
      fetchQuizInfo();
    }
  }, [currentUser, accessToken, apiBaseUrl, quizId, weekNumber]);

  if (authLoading || loading) {
    return (
      <Layout title="Loading Quiz...">
        <main className="container margin-vert--lg">
          <div className="text--center">
            <div
              style={{
                width: '48px',
                height: '48px',
                border: '4px solid #e0e0e0',
                borderTopColor: 'var(--ifm-color-primary)',
                borderRadius: '50%',
                margin: '0 auto 1rem',
                animation: 'spin 1s linear infinite',
              }}
            />
            <style>
              {`@keyframes spin { to { transform: rotate(360deg); } }`}
            </style>
            <p>Loading quiz...</p>
          </div>
        </main>
      </Layout>
    );
  }

  if (!currentUser) {
    return null;
  }

  if (error) {
    return (
      <Layout title="Quiz Error">
        <main className="container margin-vert--lg">
          <div className="text--center">
            <h2>Unable to Load Quiz</h2>
            <p style={{ color: 'var(--ifm-color-danger)' }}>{error}</p>
            <button
              onClick={() => history.goBack()}
              className="button button--secondary"
            >
              Go Back
            </button>
          </div>
        </main>
      </Layout>
    );
  }

  if (!quizInfo) {
    return (
      <Layout title="Quiz Not Found">
        <main className="container margin-vert--lg">
          <div className="text--center">
            <h2>Quiz Not Found</h2>
            <p>The requested quiz could not be found.</p>
            <button
              onClick={() => history.push('/assessments')}
              className="button button--primary"
            >
              View All Assessments
            </button>
          </div>
        </main>
      </Layout>
    );
  }

  // If started, show the quiz player
  if (started) {
    return (
      <Layout title={quizInfo.title}>
        <main className="container margin-vert--lg">
          <QuizPlayer quizId={quizInfo.id} weekNumber={quizInfo.week_number} />
        </main>
      </Layout>
    );
  }

  // Show quiz info and start button
  const attemptsRemaining =
    quizInfo.max_attempts - (quizInfo.user_attempts_count || 0);

  return (
    <Layout
      title={quizInfo.title}
      description={quizInfo.description}
    >
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div
              style={{
                background: 'var(--ifm-background-surface-color)',
                borderRadius: '12px',
                padding: '2rem',
                boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
              }}
            >
              <div style={{ marginBottom: '1.5rem' }}>
                <span
                  style={{
                    background: 'var(--ifm-color-primary-lightest)',
                    color: 'var(--ifm-color-primary-darker)',
                    padding: '0.25rem 0.75rem',
                    borderRadius: '1rem',
                    fontSize: '0.875rem',
                    fontWeight: 500,
                  }}
                >
                  Week {quizInfo.week_number} - {quizInfo.chapter}
                </span>
              </div>

              <h1 style={{ marginBottom: '0.5rem' }}>{quizInfo.title}</h1>

              {quizInfo.description && (
                <p style={{ color: 'var(--ifm-color-emphasis-700)', fontSize: '1.1rem' }}>
                  {quizInfo.description}
                </p>
              )}

              <hr style={{ margin: '1.5rem 0' }} />

              <div
                style={{
                  display: 'grid',
                  gridTemplateColumns: 'repeat(auto-fit, minmax(150px, 1fr))',
                  gap: '1rem',
                  marginBottom: '1.5rem',
                }}
              >
                <div
                  style={{
                    padding: '1rem',
                    background: 'var(--ifm-color-emphasis-100)',
                    borderRadius: '8px',
                    textAlign: 'center',
                  }}
                >
                  <div style={{ fontSize: '1.5rem', fontWeight: 700, color: 'var(--ifm-color-primary)' }}>
                    {quizInfo.question_count}
                  </div>
                  <div style={{ fontSize: '0.875rem', color: 'var(--ifm-color-emphasis-600)' }}>
                    Questions
                  </div>
                </div>

                <div
                  style={{
                    padding: '1rem',
                    background: 'var(--ifm-color-emphasis-100)',
                    borderRadius: '8px',
                    textAlign: 'center',
                  }}
                >
                  <div style={{ fontSize: '1.5rem', fontWeight: 700, color: 'var(--ifm-color-primary)' }}>
                    {quizInfo.passing_score}%
                  </div>
                  <div style={{ fontSize: '0.875rem', color: 'var(--ifm-color-emphasis-600)' }}>
                    Passing Score
                  </div>
                </div>

                <div
                  style={{
                    padding: '1rem',
                    background: attemptsRemaining > 0
                      ? 'var(--ifm-color-success-lightest)'
                      : 'var(--ifm-color-danger-lightest)',
                    borderRadius: '8px',
                    textAlign: 'center',
                  }}
                >
                  <div
                    style={{
                      fontSize: '1.5rem',
                      fontWeight: 700,
                      color: attemptsRemaining > 0
                        ? 'var(--ifm-color-success)'
                        : 'var(--ifm-color-danger)',
                    }}
                  >
                    {attemptsRemaining}
                  </div>
                  <div style={{ fontSize: '0.875rem', color: 'var(--ifm-color-emphasis-600)' }}>
                    Attempts Left
                  </div>
                </div>
              </div>

              {quizInfo.user_best_score !== null && quizInfo.user_best_score !== undefined && (
                <div
                  style={{
                    padding: '1rem',
                    background: 'var(--ifm-color-info-lightest)',
                    borderRadius: '8px',
                    marginBottom: '1.5rem',
                  }}
                >
                  <strong>Your Best Score:</strong> {Math.round(quizInfo.user_best_score)}%
                  {quizInfo.user_best_score >= quizInfo.passing_score ? (
                    <span style={{ color: 'var(--ifm-color-success)', marginLeft: '0.5rem' }}>
                      (Passed)
                    </span>
                  ) : (
                    <span style={{ color: 'var(--ifm-color-warning-darker)', marginLeft: '0.5rem' }}>
                      (Not Passed)
                    </span>
                  )}
                </div>
              )}

              <div style={{ display: 'flex', gap: '1rem', justifyContent: 'center' }}>
                <button
                  onClick={() => history.goBack()}
                  className="button button--secondary button--lg"
                >
                  Go Back
                </button>

                {attemptsRemaining > 0 ? (
                  <button
                    onClick={() => setStarted(true)}
                    className="button button--primary button--lg"
                  >
                    Start Quiz
                  </button>
                ) : (
                  <button
                    disabled
                    className="button button--secondary button--lg"
                    style={{ cursor: 'not-allowed', opacity: 0.6 }}
                  >
                    No Attempts Remaining
                  </button>
                )}
              </div>

              <div
                style={{
                  marginTop: '1.5rem',
                  padding: '1rem',
                  background: 'var(--ifm-color-warning-lightest)',
                  borderRadius: '8px',
                  fontSize: '0.875rem',
                  color: 'var(--ifm-color-warning-contrast-foreground, #333)',
                }}
              >
                <strong style={{ color: '#5a4a00' }}>Before you start:</strong>
                <ul style={{ margin: '0.5rem 0 0 1rem', padding: 0, color: '#5a4a00' }}>
                  <li>Make sure you have a stable internet connection</li>
                  <li>Your answers are auto-saved as you go</li>
                  <li>You can navigate between questions freely</li>
                  <li>Click "Submit Quiz" on the last question when done</li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
