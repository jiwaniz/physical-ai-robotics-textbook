import React from 'react';
import styles from './Quiz.module.css';

interface QuizProgressProps {
  current: number;
  total: number;
  answered: number;
}

const QuizProgress: React.FC<QuizProgressProps> = ({ current, total, answered }) => {
  const percentage = Math.round((answered / total) * 100);

  return (
    <div className={styles.progress}>
      <div className={styles.progressText}>
        <span>
          Question {current} of {total}
        </span>
        <span className={styles.progressAnswered}>
          {answered} answered ({percentage}%)
        </span>
      </div>
      <div className={styles.progressBar}>
        <div className={styles.progressFill} style={{ width: `${percentage}%` }} />
      </div>
    </div>
  );
};

export default QuizProgress;
