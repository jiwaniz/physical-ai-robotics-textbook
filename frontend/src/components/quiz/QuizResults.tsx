import React from 'react';
import Link from '@docusaurus/Link';
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
          <h2>{passed ? 'Congratulations!' : 'Keep Practicing!'}</h2>
          <div className={styles.resultsIcon}>{passed ? '✅' : '📚'}</div>
        </div>

        <div className={styles.resultsScore}>
          <div className={styles.scoreCircle}>
            <span className={styles.scorePercentage}>{Math.round(results.percentage)}%</span>
            <span className={styles.scoreLabel}>Score</span>
          </div>
        </div>

        <div className={styles.resultsDetails}>
          <div className={styles.resultsStat}>
            <span className={styles.statLabel}>Points Earned</span>
            <span className={styles.statValue}>
              {results.score} / {results.max_score}
            </span>
          </div>

          <div className={styles.resultsStat}>
            <span className={styles.statLabel}>Passing Score</span>
            <span className={styles.statValue}>{results.passing_score}%</span>
          </div>

          <div className={styles.resultsStat}>
            <span className={styles.statLabel}>Status</span>
            <span className={`${styles.statValue} ${passed ? styles.statusPassed : styles.statusFailed}`}>
              {passed ? 'PASSED' : 'NOT PASSED'}
            </span>
          </div>

          {results.pending_grading_count > 0 && (
            <div className={styles.resultsNote}>
              <strong>Note:</strong> {results.pending_grading_count} question(s) require manual
              grading. Your final score may change after review.
            </div>
          )}
        </div>

        <div className={styles.resultsActions}>
          <Link
            to={`/docs/0${Math.floor(weekNumber / 3)}-${getChapterSlug(weekNumber)}/week-0${weekNumber}`}
            className="button button--secondary"
          >
            Review Content
          </Link>
          <Link to="/assessments" className="button button--primary">
            View All Assessments
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
