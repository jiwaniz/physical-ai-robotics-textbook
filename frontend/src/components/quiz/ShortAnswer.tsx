import React, { useState, useEffect } from 'react';
import { useQuiz } from './QuizContext';
import styles from './Quiz.module.css';

interface ShortAnswerProps {
  questionId: number;
  questionText: string;
  codeSnippet?: string;
  maxLength?: number;
}

const ShortAnswer: React.FC<ShortAnswerProps> = ({
  questionId,
  questionText,
  codeSnippet,
  maxLength = 500,
}) => {
  const { state, saveAnswer } = useQuiz();
  const currentAnswer = state.answers.get(questionId);
  const [text, setText] = useState((currentAnswer?.response?.text as string) || '');

  useEffect(() => {
    // Debounce save
    const timer = setTimeout(() => {
      if (text.trim()) {
        saveAnswer(questionId, { text });
      }
    }, 500);
    return () => clearTimeout(timer);
  }, [text, questionId, saveAnswer]);

  const handleChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const newValue = e.target.value;
    if (newValue.length <= maxLength) {
      setText(newValue);
    }
  };

  return (
    <div className={styles.questionShortAnswer}>
      <p className={styles.questionText}>{questionText}</p>

      {codeSnippet && (
        <pre className={styles.codeSnippet}>
          <code>{codeSnippet}</code>
        </pre>
      )}

      <div className={styles.textareaContainer}>
        <textarea
          className={styles.answerTextarea}
          value={text}
          onChange={handleChange}
          placeholder="Type your answer here..."
          rows={6}
        />
        <div className={styles.charCount}>
          {text.length} / {maxLength} characters
        </div>
      </div>
    </div>
  );
};

export default ShortAnswer;
