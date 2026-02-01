import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import Translate, { translate } from '@docusaurus/Translate';
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
          throw new Error(translate({ message: 'Failed to load quiz attempts', id: 'page.myScores.error.failedLoadAttempts' }));
        }

        const data = await response.json();
        setAttempts(data);
      } catch (err) {
        setError(err instanceof Error ? err.message : translate({ message: 'Failed to load attempts', id: 'page.myScores.error.failedLoad' }));
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
      <Layout title={translate({ message: 'My Scores', id: 'page.myScores.layoutTitle' })}>
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
            <p><Translate id="page.myScores.loadingScores">Loading your scores...</Translate></p>
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
      <Layout title={translate({ message: 'My Scores', id: 'page.myScores.layoutTitle' })}>
        <main className="container margin-vert--lg">
          <div className="text--center">
            <h2><Translate id="page.myScores.errorHeading">Error Loading Scores</Translate></h2>
            <p style={{ color: 'var(--ifm-color-danger)' }}>{error}</p>
            <button
              onClick={() => window.location.reload()}
              className="button button--primary"
            >
              <Translate id="page.myScores.tryAgain">Try Again</Translate>
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
    <Layout title={translate({ message: 'My Scores', id: 'page.myScores.layoutTitle' })} description={translate({ message: 'View your quiz scores and progress', id: 'page.myScores.layoutDescription' })}>
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <h1><Translate id="page.myScores.heading">My Quiz Scores</Translate></h1>
            <p style={{ color: 'var(--ifm-color-emphasis-700)', marginBottom: '2rem' }}>
              <Translate id="page.myScores.subheading">Track your progress across all weekly quizzes.</Translate>
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
                <div style={{ color: 'var(--ifm-color-emphasis-700)' }}><Translate id="page.myScores.stat.quizzesAttempted">Quizzes Attempted</Translate></div>
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
                <div style={{ color: 'var(--ifm-color-emphasis-700)' }}><Translate id="page.myScores.stat.quizzesPassed">Quizzes Passed</Translate></div>
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
                <div style={{ color: 'var(--ifm-color-emphasis-700)' }}><Translate id="page.myScores.stat.totalAttempts">Total Attempts</Translate></div>
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
                <h3><Translate id="page.myScores.empty.heading">No Quiz Attempts Yet</Translate></h3>
                <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
                  <Translate id="page.myScores.empty.description">You haven't taken any quizzes yet. Start learning and test your knowledge!</Translate>
                </p>
                <Link to="/assessments" className="button button--primary button--lg">
                  <Translate id="page.myScores.empty.viewQuizzes">View Available Quizzes</Translate>
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
                            <span style={{ color: 'var(--ifm-color-emphasis-600)' }}><Translate id="page.myScores.bestScore">Best Score:</Translate></span>
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
                              <th style={{ padding: '0.75rem 1.5rem', textAlign: 'left' }}><Translate id="page.myScores.table.attempt">Attempt</Translate></th>
                              <th style={{ padding: '0.75rem 1.5rem', textAlign: 'left' }}><Translate id="page.myScores.table.date">Date</Translate></th>
                              <th style={{ padding: '0.75rem 1.5rem', textAlign: 'center' }}><Translate id="page.myScores.table.score">Score</Translate></th>
                              <th style={{ padding: '0.75rem 1.5rem', textAlign: 'center' }}><Translate id="page.myScores.table.status">Status</Translate></th>
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
                                      : formatDate(attempt.started_at) + ' (' + translate({ message: 'In Progress', id: 'page.myScores.status.inProgress' }) + ')'}
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
                                        <Translate id="page.myScores.status.inProgress">In Progress</Translate>
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
                                        <Translate id="page.myScores.status.passed">Passed</Translate>
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
                                        <Translate id="page.myScores.status.notPassed">Not Passed</Translate>
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
                <Translate id="page.myScores.viewAllAssessments">View All Assessments</Translate>
              </Link>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
