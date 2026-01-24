import React from 'react';
import { Question } from './QuizContext';
import MultipleChoice from './MultipleChoice';
import ShortAnswer from './ShortAnswer';
import CodeCompletion from './CodeCompletion';
import styles from './Quiz.module.css';

interface QuizQuestionProps {
  question: Question;
  questionNumber: number;
}

const QuizQuestion: React.FC<QuizQuestionProps> = ({ question, questionNumber }) => {
  const content = question.content as Record<string, unknown>;

  const renderQuestion = () => {
    switch (question.question_type) {
      case 'multiple_choice':
        return (
          <MultipleChoice
            questionId={question.id}
            questionText={content.question_text as string}
            codeSnippet={content.code_snippet as string | undefined}
            options={content.options as Array<{ id: string; text: string }>}
          />
        );

      case 'short_answer':
        return (
          <ShortAnswer
            questionId={question.id}
            questionText={content.question_text as string}
            codeSnippet={content.code_snippet as string | undefined}
            maxLength={content.max_length as number | undefined}
          />
        );

      case 'code_completion':
        return (
          <CodeCompletion
            questionId={question.id}
            questionText={content.question_text as string}
            codeTemplate={content.code_template as string}
            language={content.language as string | undefined}
            hints={content.hints as string[] | undefined}
          />
        );

      default:
        return <p>Unknown question type</p>;
    }
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
