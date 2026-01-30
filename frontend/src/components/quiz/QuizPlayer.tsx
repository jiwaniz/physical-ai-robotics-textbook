import React, { useState, useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import { useQuiz, QuizProvider, QuizResult } from './QuizContext';
import QuizQuestion from './QuizQuestion';
import QuizProgress from './QuizProgress';
import QuizResults from './QuizResults';
import styles from './Quiz.module.css';

interface QuizPlayerInnerProps {
  quizId: number;
  weekNumber: number;
}

const QuizPlayerInner: React.FC<QuizPlayerInnerProps> = ({ quizId, weekNumber }) => {
  const {
    state,
    startQuiz,
    submitQuiz,
    goToQuestion,
    nextQuestion,
    previousQuestion,
  } = useQuiz();
  const [results, setResults] = useState<QuizResult | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const history = useHistory();

  useEffect(() => {
    setLoading(true);
    startQuiz(quizId)
      .then(() => setLoading(false))
      .catch((err) => {
        setError(err.message);
        setLoading(false);
      });
  }, [quizId, startQuiz]);

  const handleSubmit = async () => {
    const unanswered = state.questions.length - state.answers.size;
    const message =
      unanswered > 0
        ? `You have ${unanswered} unanswered question(s). Are you sure you want to submit?`
        : 'Are you sure you want to submit? You cannot change your answers after submission.';

    const confirmed = window.confirm(message);
    if (!confirmed) return;

    try {
      const result = await submitQuiz();
      setResults(result);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to submit quiz');
    }
  };

  if (loading) {
    return (
      <div className={styles.quizLoading}>
        <div className={styles.spinner}></div>
        <p>Loading quiz...</p>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.quizError}>
        <h3>Error</h3>
        <p>{error}</p>
        <button onClick={() => history.goBack()} className="button button--secondary">
          Go Back
        </button>
      </div>
    );
  }

  if (results) {
    return <QuizResults results={results} weekNumber={weekNumber} />;
  }

  if (!state.attemptId || state.questions.length === 0) {
    return (
      <div className={styles.quizError}>
        <h3>Error</h3>
        <p>Failed to load quiz questions</p>
        <button onClick={() => history.goBack()} className="button button--secondary">
          Go Back
        </button>
      </div>
    );
  }

  const currentQuestion = state.questions[state.currentQuestionIndex];
  const isFirstQuestion = state.currentQuestionIndex === 0;
  const isLastQuestion = state.currentQuestionIndex === state.questions.length - 1;
  const answeredCount = state.answers.size;

  return (
    <div className={styles.quizPlayer}>
      <div className={styles.quizHeader}>
        <QuizProgress
          current={state.currentQuestionIndex + 1}
          total={state.questions.length}
          answered={answeredCount}
        />
      </div>

      <div className={styles.quizBody}>
        <QuizQuestion
          question={currentQuestion}
          questionNumber={state.currentQuestionIndex + 1}
        />
      </div>

      <div className={styles.quizNavigation}>
        <button
          onClick={previousQuestion}
          disabled={isFirstQuestion}
          className="button button--secondary"
        >
          Previous
        </button>

        <div className={styles.questionDots}>
          {state.questions.map((q, idx) => (
            <button
              key={q.id}
              onClick={() => goToQuestion(idx)}
              className={`${styles.questionDot} ${
                idx === state.currentQuestionIndex ? styles.active : ''
              } ${state.answers.has(q.id) ? styles.answered : ''}`}
              title={`Question ${idx + 1}${state.answers.has(q.id) ? ' (answered)' : ''}`}
            >
              {idx + 1}
            </button>
          ))}
        </div>

        {isLastQuestion ? (
          <button
            onClick={handleSubmit}
            disabled={state.isSubmitting}
            className="button button--primary"
          >
            {state.isSubmitting ? 'Submitting...' : 'Submit Quiz'}
          </button>
        ) : (
          <button onClick={nextQuestion} className="button button--primary">
            Next
          </button>
        )}
      </div>
    </div>
  );
};

interface QuizPlayerProps {
  quizId: number;
  weekNumber: number;
}

const QuizPlayer: React.FC<QuizPlayerProps> = (props) => (
  <QuizProvider>
    <QuizPlayerInner {...props} />
  </QuizProvider>
);

export default QuizPlayer;
