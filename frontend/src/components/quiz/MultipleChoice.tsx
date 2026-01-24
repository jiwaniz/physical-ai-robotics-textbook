import React from 'react';
import { useQuiz } from './QuizContext';
import styles from './Quiz.module.css';

interface Option {
  id: string;
  text: string;
}

interface MultipleChoiceProps {
  questionId: number;
  questionText: string;
  codeSnippet?: string;
  options: Option[];
}

const MultipleChoice: React.FC<MultipleChoiceProps> = ({
  questionId,
  questionText,
  codeSnippet,
  options,
}) => {
  const { state, saveAnswer } = useQuiz();
  const currentAnswer = state.answers.get(questionId);
  const selectedOption = (currentAnswer?.response?.selected_option as string) || '';

  const handleSelect = (optionId: string) => {
    saveAnswer(questionId, { selected_option: optionId });
  };

  return (
    <div className={styles.questionMultipleChoice}>
      <p className={styles.questionText}>{questionText}</p>

      {codeSnippet && (
        <pre className={styles.codeSnippet}>
          <code>{codeSnippet}</code>
        </pre>
      )}

      <div className={styles.optionsList}>
        {options.map((option) => (
          <label
            key={option.id}
            className={`${styles.optionItem} ${selectedOption === option.id ? styles.selected : ''}`}
          >
            <input
              type="radio"
              name={`question-${questionId}`}
              value={option.id}
              checked={selectedOption === option.id}
              onChange={() => handleSelect(option.id)}
              className={styles.optionRadio}
            />
            <span className={styles.optionLabel}>{option.id.toUpperCase()}.</span>
            <span className={styles.optionText}>{option.text}</span>
          </label>
        ))}
      </div>
    </div>
  );
};

export default MultipleChoice;
