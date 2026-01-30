import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../components/AuthContext';

interface QuizAttempt {
  id: number;
  quiz_id: number;
  quiz_title: string;
  attempt_number: number;
  started_at: string;
  submitted_at: string | null;
  score: number | null;
  max_score: number | null;
  percentage: number | null;
  is_submitted: boolean;
  is_fully_graded: boolean;
}

export default function MyScoresPage(): JSX.Element {
  const { currentUser, isLoading: authLoading, accessToken, apiBaseUrl } = useAuth();
  const history = useHistory();
  const [attempts, setAttempts] = useState<QuizAttempt[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!authLoading && !currentUser) {
      history.push('/signin?redirect=/my-scores');
    }
  }, [currentUser, authLoading, history]);

  useEffect(() => {
    const fetchAttempts = async () => {
      if (!accessToken) return;

      setLoading(true);
      setError(null);

      try {
        const response = await fetch(`${apiBaseUrl}/api/quizzes/attempts`, {
          headers: {
            Authorization: `Bearer ${accessToken}`,
          },
          credentials: 'include',
        });

        if (!response.ok) {
          throw new Error('Failed to load quiz attempts');
        }

        const data = await response.json();
        setAttempts(data);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Failed to load attempts');
      } finally {
        setLoading(false);
      }
    };

    if (currentUser && accessToken) {
      fetchAttempts();
    }
  }, [currentUser, accessToken, apiBaseUrl]);

  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString('en-US', {
      year: 'numeric',
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit',
    });
  };

  const getPassingScore = () => 60; // Default passing score

  if (authLoading || loading) {
    return (
      <Layout title="My Scores">
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
            <p>Loading your scores...</p>
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
      <Layout title="My Scores">
        <main className="container margin-vert--lg">
          <div className="text--center">
            <h2>Error Loading Scores</h2>
            <p style={{ color: 'var(--ifm-color-danger)' }}>{error}</p>
            <button
              onClick={() => window.location.reload()}
              className="button button--primary"
            >
              Try Again
            </button>
          </div>
        </main>
      </Layout>
    );
  }

  // Group attempts by quiz
  const attemptsByQuiz = attempts.reduce((acc, attempt) => {
    if (!acc[attempt.quiz_id]) {
      acc[attempt.quiz_id] = {
        quiz_title: attempt.quiz_title,
        attempts: [],
        best_score: null as number | null,
      };
    }
    acc[attempt.quiz_id].attempts.push(attempt);
    if (attempt.percentage !== null) {
      if (acc[attempt.quiz_id].best_score === null || attempt.percentage > acc[attempt.quiz_id].best_score) {
        acc[attempt.quiz_id].best_score = attempt.percentage;
      }
    }
    return acc;
  }, {} as Record<number, { quiz_title: string; attempts: QuizAttempt[]; best_score: number | null }>);

  const totalAttempts = attempts.filter(a => a.is_submitted).length;
  const passedAttempts = attempts.filter(a => a.is_submitted && a.percentage !== null && a.percentage >= getPassingScore()).length;

  return (
    <Layout title="My Scores" description="View your quiz scores and progress">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <h1>My Quiz Scores</h1>
            <p style={{ color: 'var(--ifm-color-emphasis-700)', marginBottom: '2rem' }}>
              Track your progress across all weekly quizzes.
            </p>

            {/* Summary Stats */}
            <div
              style={{
                display: 'grid',
                gridTemplateColumns: 'repeat(auto-fit, minmax(200px, 1fr))',
                gap: '1rem',
                marginBottom: '2rem',
              }}
            >
              <div
                style={{
                  padding: '1.5rem',
                  background: 'var(--ifm-color-primary-lightest)',
                  borderRadius: '12px',
                  textAlign: 'center',
                }}
              >
                <div style={{ fontSize: '2rem', fontWeight: 700, color: 'var(--ifm-color-primary)' }}>
                  {Object.keys(attemptsByQuiz).length}
                </div>
                <div style={{ color: 'var(--ifm-color-emphasis-700)' }}>Quizzes Attempted</div>
              </div>

              <div
                style={{
                  padding: '1.5rem',
                  background: 'var(--ifm-color-success-lightest)',
                  borderRadius: '12px',
                  textAlign: 'center',
                }}
              >
                <div style={{ fontSize: '2rem', fontWeight: 700, color: 'var(--ifm-color-success)' }}>
                  {passedAttempts}
                </div>
                <div style={{ color: 'var(--ifm-color-emphasis-700)' }}>Quizzes Passed</div>
              </div>

              <div
                style={{
                  padding: '1.5rem',
                  background: 'var(--ifm-color-emphasis-100)',
                  borderRadius: '12px',
                  textAlign: 'center',
                }}
              >
                <div style={{ fontSize: '2rem', fontWeight: 700, color: 'var(--ifm-color-emphasis-800)' }}>
                  {totalAttempts}
                </div>
                <div style={{ color: 'var(--ifm-color-emphasis-700)' }}>Total Attempts</div>
              </div>
            </div>

            {attempts.length === 0 ? (
              <div
                style={{
                  textAlign: 'center',
                  padding: '3rem',
                  background: 'var(--ifm-background-surface-color)',
                  borderRadius: '12px',
                  boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
                }}
              >
                <div style={{ fontSize: '3rem', marginBottom: '1rem' }}>📝</div>
                <h3>No Quiz Attempts Yet</h3>
                <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                  You haven't taken any quizzes yet. Start learning and test your knowledge!
                </p>
                <Link to="/assessments" className="button button--primary button--lg">
                  View Available Quizzes
                </Link>
              </div>
            ) : (
              <div style={{ display: 'flex', flexDirection: 'column', gap: '1.5rem' }}>
                {Object.entries(attemptsByQuiz)
                  .sort(([, a], [, b]) => {
                    // Sort by quiz title (which includes week number)
                    return a.quiz_title.localeCompare(b.quiz_title);
                  })
                  .map(([quizId, data]) => (
                    <div
                      key={quizId}
                      style={{
                        background: 'var(--ifm-background-surface-color)',
                        borderRadius: '12px',
                        boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
                        overflow: 'hidden',
                      }}
                    >
                      {/* Quiz Header */}
                      <div
                        style={{
                          padding: '1rem 1.5rem',
                          background: 'var(--ifm-color-emphasis-100)',
                          display: 'flex',
                          justifyContent: 'space-between',
                          alignItems: 'center',
                          flexWrap: 'wrap',
                          gap: '1rem',
                        }}
                      >
                        <h3 style={{ margin: 0 }}>{data.quiz_title}</h3>
                        {data.best_score !== null && (
                          <div
                            style={{
                              display: 'flex',
                              alignItems: 'center',
                              gap: '0.5rem',
                            }}
                          >
                            <span style={{ color: 'var(--ifm-color-emphasis-600)' }}>Best Score:</span>
                            <span
                              style={{
                                fontWeight: 700,
                                color: data.best_score >= getPassingScore()
                                  ? 'var(--ifm-color-success)'
                                  : 'var(--ifm-color-danger)',
                              }}
                            >
                              {Math.round(data.best_score)}%
                              {data.best_score >= getPassingScore() ? ' ✓' : ''}
                            </span>
                          </div>
                        )}
                      </div>

                      {/* Attempts Table */}
                      <div style={{ overflowX: 'auto' }}>
                        <table style={{ width: '100%', borderCollapse: 'collapse' }}>
                          <thead>
                            <tr style={{ borderBottom: '1px solid var(--ifm-color-emphasis-200)' }}>
                              <th style={{ padding: '0.75rem 1.5rem', textAlign: 'left' }}>Attempt</th>
                              <th style={{ padding: '0.75rem 1.5rem', textAlign: 'left' }}>Date</th>
                              <th style={{ padding: '0.75rem 1.5rem', textAlign: 'center' }}>Score</th>
                              <th style={{ padding: '0.75rem 1.5rem', textAlign: 'center' }}>Status</th>
                            </tr>
                          </thead>
                          <tbody>
                            {data.attempts
                              .sort((a, b) => b.attempt_number - a.attempt_number)
                              .map((attempt) => (
                                <tr
                                  key={attempt.id}
                                  style={{ borderBottom: '1px solid var(--ifm-color-emphasis-100)' }}
                                >
                                  <td style={{ padding: '0.75rem 1.5rem' }}>
                                    #{attempt.attempt_number}
                                  </td>
                                  <td style={{ padding: '0.75rem 1.5rem', color: 'var(--ifm-color-emphasis-600)' }}>
                                    {attempt.submitted_at
                                      ? formatDate(attempt.submitted_at)
                                      : formatDate(attempt.started_at) + ' (In Progress)'}
                                  </td>
                                  <td style={{ padding: '0.75rem 1.5rem', textAlign: 'center' }}>
                                    {attempt.is_submitted && attempt.percentage !== null ? (
                                      <span
                                        style={{
                                          fontWeight: 700,
                                          color: attempt.percentage >= getPassingScore()
                                            ? 'var(--ifm-color-success)'
                                            : 'var(--ifm-color-danger)',
                                        }}
                                      >
                                        {Math.round(attempt.percentage)}%
                                        <span style={{ fontWeight: 400, color: 'var(--ifm-color-emphasis-500)', marginLeft: '0.5rem' }}>
                                          ({attempt.score}/{attempt.max_score})
                                        </span>
                                      </span>
                                    ) : (
                                      <span style={{ color: 'var(--ifm-color-emphasis-500)' }}>—</span>
                                    )}
                                  </td>
                                  <td style={{ padding: '0.75rem 1.5rem', textAlign: 'center' }}>
                                    {!attempt.is_submitted ? (
                                      <span
                                        style={{
                                          padding: '0.25rem 0.75rem',
                                          borderRadius: '1rem',
                                          fontSize: '0.8rem',
                                          background: 'var(--ifm-color-warning-lightest)',
                                          color: '#5a4a00',
                                        }}
                                      >
                                        In Progress
                                      </span>
                                    ) : attempt.percentage !== null && attempt.percentage >= getPassingScore() ? (
                                      <span
                                        style={{
                                          padding: '0.25rem 0.75rem',
                                          borderRadius: '1rem',
                                          fontSize: '0.8rem',
                                          background: 'var(--ifm-color-success-lightest)',
                                          color: 'var(--ifm-color-success-darkest)',
                                        }}
                                      >
                                        Passed
                                      </span>
                                    ) : (
                                      <span
                                        style={{
                                          padding: '0.25rem 0.75rem',
                                          borderRadius: '1rem',
                                          fontSize: '0.8rem',
                                          background: 'var(--ifm-color-danger-lightest)',
                                          color: 'var(--ifm-color-danger-darkest)',
                                        }}
                                      >
                                        Not Passed
                                      </span>
                                    )}
                                  </td>
                                </tr>
                              ))}
                          </tbody>
                        </table>
                      </div>
                    </div>
                  ))}
              </div>
            )}

            {/* Back to Assessments */}
            <div style={{ marginTop: '2rem', textAlign: 'center' }}>
              <Link to="/assessments" className="button button--secondary">
                View All Assessments
              </Link>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
