import React, { useState, useEffect, useCallback } from 'react';
import styles from './Quiz.module.css';

interface QuizTimerProps {
  startedAt: Date;
  timeLimitMinutes: number;
  onTimeUp: () => void;
}

const QuizTimer: React.FC<QuizTimerProps> = ({ startedAt, timeLimitMinutes, onTimeUp }) => {
  const calculateTimeRemaining = useCallback(() => {
    const now = new Date();
    const elapsed = (now.getTime() - startedAt.getTime()) / 1000;
    const totalSeconds = timeLimitMinutes * 60;
    return Math.max(0, Math.floor(totalSeconds - elapsed));
  }, [startedAt, timeLimitMinutes]);

  const [secondsRemaining, setSecondsRemaining] = useState(calculateTimeRemaining);

  useEffect(() => {
    const timer = setInterval(() => {
      const remaining = calculateTimeRemaining();
      setSecondsRemaining(remaining);

      if (remaining <= 0) {
        clearInterval(timer);
        onTimeUp();
      }
    }, 1000);

    return () => clearInterval(timer);
  }, [calculateTimeRemaining, onTimeUp]);

  const minutes = Math.floor(secondsRemaining / 60);
  const seconds = secondsRemaining % 60;

  const isLowTime = secondsRemaining < 300; // Less than 5 minutes
  const isCriticalTime = secondsRemaining < 60; // Less than 1 minute

  return (
    <div
      className={`${styles.timer} ${isLowTime ? styles.timerLow : ''} ${isCriticalTime ? styles.timerCritical : ''}`}
    >
      <span className={styles.timerIcon}>&#x23F1;</span>
      <span className={styles.timerDisplay}>
        {String(minutes).padStart(2, '0')}:{String(seconds).padStart(2, '0')}
      </span>
    </div>
  );
};

export default QuizTimer;
