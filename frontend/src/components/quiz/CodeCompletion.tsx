import React, { useState, useEffect } from 'react';
import { useQuiz } from './QuizContext';
import styles from './Quiz.module.css';

interface CodeCompletionProps {
  questionId: number;
  questionText: string;
  codeTemplate: string;
  language?: string;
  hints?: string[];
}

const CodeCompletion: React.FC<CodeCompletionProps> = ({
  questionId,
  questionText,
  codeTemplate,
  language = 'python',
  hints = [],
}) => {
  const { state, saveAnswer } = useQuiz();
  const currentAnswer = state.answers.get(questionId);
  const [code, setCode] = useState((currentAnswer?.response?.code as string) || codeTemplate);
  const [showHints, setShowHints] = useState(false);

  useEffect(() => {
    // Debounce save
    const timer = setTimeout(() => {
      if (code !== codeTemplate) {
        saveAnswer(questionId, { code });
      }
    }, 500);
    return () => clearTimeout(timer);
  }, [code, questionId, saveAnswer, codeTemplate]);

  const handleChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setCode(e.target.value);
  };

  return (
    <div className={styles.questionCodeCompletion}>
      <p className={styles.questionText}>{questionText}</p>

      <div className={styles.codeEditorContainer}>
        <div className={styles.codeEditorHeader}>
          <span className={styles.languageBadge}>{language}</span>
          {hints.length > 0 && (
            <button
              type="button"
              className={styles.hintToggle}
              onClick={() => setShowHints(!showHints)}
            >
              {showHints ? 'Hide Hints' : 'Show Hints'}
            </button>
          )}
        </div>

        <textarea
          className={styles.codeEditor}
          value={code}
          onChange={handleChange}
          spellCheck={false}
          rows={12}
        />

        {showHints && hints.length > 0 && (
          <div className={styles.hintsContainer}>
            <strong>Hints:</strong>
            <ul>
              {hints.map((hint, idx) => (
                <li key={idx}>{hint}</li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </div>
  );
};

export default CodeCompletion;
