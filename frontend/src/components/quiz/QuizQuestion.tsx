import React from 'react';
import { Question } from './QuizContext';
import MultipleChoice from './MultipleChoice';
import styles from './Quiz.module.css';

interface QuizQuestionProps {
  question: Question;
  questionNumber: number;
}

const QuizQuestion: React.FC<QuizQuestionProps> = ({ question, questionNumber }) => {
  const content = question.content as Record<string, unknown>;

  const renderQuestion = () => {
    if (question.question_type === 'multiple_choice') {
      return (
        <MultipleChoice
          questionId={question.id}
          questionText={content.question_text as string}
          codeSnippet={content.code_snippet as string | undefined}
          options={content.options as Array<{ id: string; text: string }>}
        />
      );
    }
    return <p>Unsupported question type: {question.question_type}</p>;
  };

  const getCategoryLabel = () => {
    switch (question.category) {
      case 'conceptual':
        return 'Conceptual';
      case 'code_comprehension':
        return 'Code Comprehension';
      case 'troubleshooting':
        return 'Troubleshooting';
      default:
        return question.category;
    }
  };

  return (
    <div className={styles.questionContainer}>
      <div className={styles.questionHeader}>
        <span className={styles.questionNumber}>Question {questionNumber}</span>
        <span className={styles.questionCategory}>{getCategoryLabel()}</span>
        <span className={styles.questionPoints}>{question.points} pts</span>
      </div>
      {renderQuestion()}
    </div>
  );
};

export default QuizQuestion;
