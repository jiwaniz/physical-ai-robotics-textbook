import React from 'react';
import Link from '@docusaurus/Link';
import Translate, { translate } from '@docusaurus/Translate';
import { QuizResult } from './QuizContext';
import styles from './Quiz.module.css';

interface QuizResultsProps {
  results: QuizResult;
  weekNumber: number;
}

const QuizResults: React.FC<QuizResultsProps> = ({ results, weekNumber }) => {
  const passed = results.passed;

  return (
    <div className={styles.resultsContainer}>
      <div className={`${styles.resultsCard} ${passed ? styles.resultsPassed : styles.resultsFailed}`}>
        <div className={styles.resultsHeader}>
          <h2>{passed
            ? translate({ message: 'Congratulations!', id: 'component.quiz.results.congratulations' })
            : translate({ message: 'Keep Practicing!', id: 'component.quiz.results.keepPracticing' })}</h2>
          <div className={styles.resultsIcon}>{passed ? '✅' : '📚'}</div>
        </div>

        <div className={styles.resultsScore}>
          <div className={styles.scoreCircle}>
            <span className={styles.scorePercentage}>{Math.round(results.percentage)}%</span>
            <span className={styles.scoreLabel}><Translate id="component.quiz.results.scoreLabel">Score</Translate></span>
          </div>
        </div>

        <div className={styles.resultsDetails}>
          <div className={styles.resultsStat}>
            <span className={styles.statLabel}><Translate id="component.quiz.results.pointsEarned">Points Earned</Translate></span>
            <span className={styles.statValue}>
              {results.score} / {results.max_score}
            </span>
          </div>

          <div className={styles.resultsStat}>
            <span className={styles.statLabel}><Translate id="component.quiz.results.passingScore">Passing Score</Translate></span>
            <span className={styles.statValue}>{results.passing_score}%</span>
          </div>

          <div className={styles.resultsStat}>
            <span className={styles.statLabel}><Translate id="component.quiz.results.statusLabel">Status</Translate></span>
            <span className={`${styles.statValue} ${passed ? styles.statusPassed : styles.statusFailed}`}>
              {passed
                ? translate({ message: 'PASSED', id: 'component.quiz.results.passed' })
                : translate({ message: 'NOT PASSED', id: 'component.quiz.results.notPassed' })}
            </span>
          </div>

          {results.pending_grading_count > 0 && (
            <div className={styles.resultsNote}>
              <strong><Translate id="component.quiz.results.noteLabel">Note:</Translate></strong>{' '}
              <Translate
                id="component.quiz.results.pendingGrading"
                values={{ count: results.pending_grading_count }}
              >
                {'{count} question(s) require manual grading. Your final score may change after review.'}
              </Translate>
            </div>
          )}
        </div>

        <div className={styles.resultsActions}>
          <Link
            to={`/docs/0${Math.floor(weekNumber / 3)}-${getChapterSlug(weekNumber)}/week-0${weekNumber}`}
            className="button button--secondary"
          >
            <Translate id="component.quiz.results.reviewContent">Review Content</Translate>
          </Link>
          <Link to="/assessments" className="button button--primary">
            <Translate id="component.quiz.results.viewAllAssessments">View All Assessments</Translate>
          </Link>
        </div>
      </div>
    </div>
  );
};

function getChapterSlug(weekNumber: number): string {
  if (weekNumber <= 2) return 'introduction';
  if (weekNumber <= 5) return 'ros2';
  if (weekNumber <= 7) return 'simulation';
  if (weekNumber <= 10) return 'isaac';
  return 'vla';
}

export default QuizResults;
