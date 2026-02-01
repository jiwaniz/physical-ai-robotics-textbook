import React from 'react';
import Translate from '@docusaurus/Translate';
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
          <Translate
            id="component.quiz.progress.questionOf"
            values={{ current, total }}
          >
            {'Question {current} of {total}'}
          </Translate>
        </span>
        <span className={styles.progressAnswered}>
          <Translate
            id="component.quiz.progress.answered"
            values={{ answered, percentage }}
          >
            {'{answered} answered ({percentage}%)'}
          </Translate>
        </span>
      </div>
      <div className={styles.progressBar}>
        <div className={styles.progressFill} style={{ width: `${percentage}%` }} />
      </div>
    </div>
  );
};

export default QuizProgress;
