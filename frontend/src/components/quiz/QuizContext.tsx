import React, { createContext, useContext, useState, useCallback, ReactNode } from 'react';
import { useAuth } from '../AuthContext';

interface Question {
  id: number;
  question_type: 'multiple_choice' | 'short_answer' | 'code_completion';
  category: string;
  points: number;
  order_index: number;
  content: Record<string, unknown>;
}

interface Answer {
  question_id: number;
  response: Record<string, unknown>;
}

interface QuizState {
  quizId: number | null;
  attemptId: number | null;
  questions: Question[];
  answers: Map<number, Answer>;
  currentQuestionIndex: number;
  startedAt: Date | null;
  timeLimit: number;
  isSubmitting: boolean;
  quizTitle: string;
}

interface QuizResult {
  attempt_id: number;
  score: number;
  max_score: number;
  percentage: number;
  is_fully_graded: boolean;
  passing_score: number;
  passed: boolean;
  auto_graded_count: number;
  pending_grading_count: number;
}

interface QuizContextType {
  state: QuizState;
  startQuiz: (quizId: number) => Promise<void>;
  saveAnswer: (questionId: number, response: Record<string, unknown>) => void;
  submitQuiz: () => Promise<QuizResult>;
  goToQuestion: (index: number) => void;
  nextQuestion: () => void;
  previousQuestion: () => void;
  resetQuiz: () => void;
}

const initialState: QuizState = {
  quizId: null,
  attemptId: null,
  questions: [],
  answers: new Map(),
  currentQuestionIndex: 0,
  startedAt: null,
  timeLimit: 20,
  isSubmitting: false,
  quizTitle: '',
};

const QuizContext = createContext<QuizContextType | undefined>(undefined);

interface QuizProviderProps {
  children: ReactNode;
}

export const QuizProvider: React.FC<QuizProviderProps> = ({ children }) => {
  const { accessToken, apiBaseUrl } = useAuth();
  const [state, setState] = useState<QuizState>(initialState);

  const startQuiz = useCallback(
    async (quizId: number) => {
      const response = await fetch(`${apiBaseUrl}/api/quizzes/${quizId}/start`, {
        method: 'POST',
        headers: {
          Authorization: `Bearer ${accessToken}`,
          'Content-Type': 'application/json',
        },
        credentials: 'include',
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Failed to start quiz');
      }

      const data = await response.json();
      setState({
        quizId: quizId,
        attemptId: data.attempt_id,
        questions: data.questions,
        answers: new Map(),
        currentQuestionIndex: 0,
        startedAt: new Date(data.started_at),
        timeLimit: data.time_limit_minutes,
        isSubmitting: false,
        quizTitle: '',
      });
    },
    [accessToken, apiBaseUrl]
  );

  const saveAnswer = useCallback(
    (questionId: number, response: Record<string, unknown>) => {
      setState((prev) => {
        const newAnswers = new Map(prev.answers);
        newAnswers.set(questionId, { question_id: questionId, response });
        return { ...prev, answers: newAnswers };
      });

      // Auto-save to server (fire and forget)
      if (state.attemptId && state.quizId) {
        fetch(
          `${apiBaseUrl}/api/quizzes/${state.quizId}/attempts/${state.attemptId}/answers`,
          {
            method: 'POST',
            headers: {
              Authorization: `Bearer ${accessToken}`,
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({ question_id: questionId, response }),
            credentials: 'include',
          }
        ).catch(console.error);
      }
    },
    [accessToken, apiBaseUrl, state.attemptId, state.quizId]
  );

  const submitQuiz = useCallback(async (): Promise<QuizResult> => {
    if (!state.attemptId || !state.quizId) {
      throw new Error('No active quiz attempt');
    }

    setState((prev) => ({ ...prev, isSubmitting: true }));

    const answers = Array.from(state.answers.values());

    const response = await fetch(
      `${apiBaseUrl}/api/quizzes/${state.quizId}/attempts/${state.attemptId}/submit`,
      {
        method: 'POST',
        headers: {
          Authorization: `Bearer ${accessToken}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ answers }),
        credentials: 'include',
      }
    );

    const result = await response.json();
    setState((prev) => ({ ...prev, isSubmitting: false }));

    if (!response.ok) {
      throw new Error(result.detail || 'Failed to submit quiz');
    }

    return result;
  }, [accessToken, apiBaseUrl, state.answers, state.attemptId, state.quizId]);

  const goToQuestion = useCallback((index: number) => {
    setState((prev) => ({ ...prev, currentQuestionIndex: index }));
  }, []);

  const nextQuestion = useCallback(() => {
    setState((prev) => ({
      ...prev,
      currentQuestionIndex: Math.min(prev.currentQuestionIndex + 1, prev.questions.length - 1),
    }));
  }, []);

  const previousQuestion = useCallback(() => {
    setState((prev) => ({
      ...prev,
      currentQuestionIndex: Math.max(prev.currentQuestionIndex - 1, 0),
    }));
  }, []);

  const resetQuiz = useCallback(() => {
    setState(initialState);
  }, []);

  return (
    <QuizContext.Provider
      value={{
        state,
        startQuiz,
        saveAnswer,
        submitQuiz,
        goToQuestion,
        nextQuestion,
        previousQuestion,
        resetQuiz,
      }}
    >
      {children}
    </QuizContext.Provider>
  );
};

export const useQuiz = (): QuizContextType => {
  const context = useContext(QuizContext);
  if (!context) {
    throw new Error('useQuiz must be used within a QuizProvider');
  }
  return context;
};

export type { Question, Answer, QuizState, QuizResult };
