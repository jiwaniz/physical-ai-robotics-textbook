import React, { useState, useRef, useEffect, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import { useAuth } from '../AuthContext';
import styles from './ChatWidget.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{ content: string; page_path: string; score: number }>;
}

interface QuizQuestion {
  id: number;
  question: string;
  options: string[];
}

interface QuizResult {
  score: number;
  total: number;
  percentage: number;
  feedback: string[];
}

type Mode = 'chat' | 'quiz';
type QuizState = 'intro' | 'loading' | 'questions' | 'results';

export default function ChatWidget(): JSX.Element | null {
  const { currentUser, accessToken, apiBaseUrl } = useAuth();
  const location = useLocation();

  // Chat state
  const [isOpen, setIsOpen] = useState(false);
  const [mode, setMode] = useState<Mode>('chat');
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversationId, setConversationId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);

  // Quiz state
  const [quizState, setQuizState] = useState<QuizState>('intro');
  const [quizQuestions, setQuizQuestions] = useState<QuizQuestion[]>([]);
  const [quizId, setQuizId] = useState<string | null>(null);
  const [currentQuestionIndex, setCurrentQuestionIndex] = useState(0);
  const [selectedAnswers, setSelectedAnswers] = useState<string[]>([]);
  const [quizResult, setQuizResult] = useState<QuizResult | null>(null);
  const [quizError, setQuizError] = useState<string | null>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Listen for text selection on the page
  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // Only capture selection if it's substantial and not in the chat widget
      if (text && text.length > 20 && text.length < 5000) {
        const anchorNode = selection?.anchorNode;
        const isInChatWidget = anchorNode?.parentElement?.closest(`.${styles.chatPanel}`);
        if (!isInChatWidget) {
          setSelectedText(text);
        }
      }
    };

    document.addEventListener('mouseup', handleSelectionChange);
    return () => document.removeEventListener('mouseup', handleSelectionChange);
  }, []);

  // Get current page content for quiz generation
  const getPageContent = useCallback((): string => {
    const article = document.querySelector('article');
    if (article) {
      return article.textContent || '';
    }
    const main = document.querySelector('main');
    return main?.textContent || '';
  }, []);

  // Send message to chatbot
  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading || !accessToken) return;

    const userMessage = inputValue.trim();
    setInputValue('');
    setMessages((prev) => [...prev, { role: 'user', content: userMessage }]);
    setIsLoading(true);

    try {
      const response = await fetch(`${apiBaseUrl}/api/chat/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${accessToken}`,
        },
        body: JSON.stringify({
          query: userMessage,
          selected_text: selectedText,
          page_path: location.pathname,
          conversation_id: conversationId,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();
      setConversationId(data.conversation_id);
      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant',
          content: data.answer,
          sources: data.sources,
        },
      ]);

      // Clear selected text after using it
      if (selectedText) {
        setSelectedText(null);
      }
    } catch (error) {
      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant',
          content: 'Sorry, I encountered an error. Please try again.',
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  // Generate quiz from current page
  const generateQuiz = async () => {
    if (!accessToken) return;

    setQuizState('loading');
    setQuizError(null);

    try {
      const pageContent = getPageContent();
      if (pageContent.length < 200) {
        setQuizError('Not enough content on this page to generate a quiz.');
        setQuizState('intro');
        return;
      }

      const response = await fetch(`${apiBaseUrl}/api/chat/quiz/generate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${accessToken}`,
        },
        body: JSON.stringify({
          page_content: pageContent.slice(0, 10000), // Limit content size
          page_path: location.pathname,
          num_questions: 5,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to generate quiz');
      }

      const data = await response.json();
      setQuizId(data.quiz_id);
      setQuizQuestions(data.questions);
      setSelectedAnswers(new Array(data.questions.length).fill(''));
      setCurrentQuestionIndex(0);
      setQuizState('questions');
    } catch (error) {
      setQuizError('Failed to generate quiz. Please try again.');
      setQuizState('intro');
    }
  };

  // Submit quiz answers
  const submitQuiz = async () => {
    if (!accessToken || !quizId) return;

    setQuizState('loading');

    try {
      const response = await fetch(`${apiBaseUrl}/api/chat/quiz/submit`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${accessToken}`,
        },
        body: JSON.stringify({
          quiz_id: quizId,
          answers: selectedAnswers,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to submit quiz');
      }

      const data = await response.json();
      setQuizResult(data);
      setQuizState('results');
    } catch (error) {
      setQuizError('Failed to submit quiz. Please try again.');
      setQuizState('questions');
    }
  };

  // Reset quiz
  const resetQuiz = () => {
    setQuizState('intro');
    setQuizQuestions([]);
    setQuizId(null);
    setSelectedAnswers([]);
    setCurrentQuestionIndex(0);
    setQuizResult(null);
    setQuizError(null);
  };

  // Handle input key press
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Clear conversation
  const clearConversation = () => {
    setMessages([]);
    setConversationId(null);
    setSelectedText(null);
  };

  // Don't render if not authenticated
  if (!currentUser) {
    return null;
  }

  return (
    <>
      {/* Chat Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        <span className={styles.chatButtonIcon}>{isOpen ? '×' : '💬'}</span>
      </button>

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.chatTitle}>
              <span>🤖</span>
              <span>AI Tutor</span>
            </div>
            <div className={styles.headerButtons}>
              <button
                className={`${styles.headerButton} ${mode === 'chat' ? styles.active : ''}`}
                onClick={() => setMode('chat')}
              >
                Chat
              </button>
              <button
                className={`${styles.headerButton} ${mode === 'quiz' ? styles.active : ''}`}
                onClick={() => {
                  setMode('quiz');
                  resetQuiz();
                }}
              >
                Quiz
              </button>
              <button
                className={styles.closeButton}
                onClick={() => setIsOpen(false)}
                aria-label="Close"
              >
                ×
              </button>
            </div>
          </div>

          {/* Chat Mode */}
          {mode === 'chat' && (
            <>
              {/* Selected Text Context */}
              {selectedText && (
                <div className={styles.selectionContext}>
                  <span className={styles.selectionContextIcon}>📝</span>
                  <div className={styles.selectionContextText}>
                    <div className={styles.selectionContextLabel}>Selected text:</div>
                    <div className={styles.selectionContextPreview}>"{selectedText}"</div>
                  </div>
                  <button
                    className={styles.clearSelection}
                    onClick={() => setSelectedText(null)}
                    aria-label="Clear selection"
                  >
                    ×
                  </button>
                </div>
              )}

              {/* Messages */}
              <div className={styles.chatMessages}>
                {messages.length === 0 && (
                  <div className={styles.welcomeMessage}>
                    <div className={styles.welcomeIcon}>👋</div>
                    <div className={styles.welcomeTitle}>Hi! I'm your AI Tutor</div>
                    <div className={styles.welcomeHints}>
                      Ask me anything about ROS 2, robotics simulation, Isaac Sim, or VLA models.
                    </div>
                    <div className={styles.selectionHint}>
                      <strong>Tip:</strong> Highlight text on any page, then ask a question about it
                      for more focused answers!
                    </div>
                  </div>
                )}

                {messages.map((msg, idx) => (
                  <div
                    key={idx}
                    className={`${styles.message} ${
                      msg.role === 'user' ? styles.userMessage : styles.assistantMessage
                    }`}
                  >
                    <div className={styles.messageContent}>{msg.content}</div>
                    {msg.sources && msg.sources.length > 0 && (
                      <div className={styles.sources}>
                        <strong>Sources:</strong>{' '}
                        {msg.sources.slice(0, 3).map((source, i) => (
                          <a
                            key={i}
                            href={source.page_path}
                            className={styles.sourceLink}
                          >
                            [{i + 1}]
                          </a>
                        ))}
                      </div>
                    )}
                  </div>
                ))}

                {isLoading && (
                  <div className={`${styles.message} ${styles.assistantMessage}`}>
                    <div className={styles.typingIndicator}>
                      <span className={styles.typingDot}></span>
                      <span className={styles.typingDot}></span>
                      <span className={styles.typingDot}></span>
                    </div>
                  </div>
                )}

                <div ref={messagesEndRef} />
              </div>

              {/* Input */}
              <div className={styles.chatInput}>
                <div className={styles.inputWrapper}>
                  <textarea
                    ref={inputRef}
                    className={styles.textInput}
                    value={inputValue}
                    onChange={(e) => setInputValue(e.target.value)}
                    onKeyPress={handleKeyPress}
                    placeholder="Ask a question..."
                    rows={1}
                    disabled={isLoading}
                  />
                </div>
                <button
                  className={styles.sendButton}
                  onClick={sendMessage}
                  disabled={!inputValue.trim() || isLoading}
                  aria-label="Send message"
                >
                  ➤
                </button>
              </div>
            </>
          )}

          {/* Quiz Mode */}
          {mode === 'quiz' && (
            <>
              <div className={styles.quizMode}>
                {quizState === 'intro' && (
                  <div className={styles.quizIntro}>
                    <div className={styles.quizIntroIcon}>📚</div>
                    <div className={styles.quizIntroTitle}>Test Your Knowledge</div>
                    <div className={styles.quizIntroDesc}>
                      Generate a 5-question quiz based on the current page content.
                    </div>
                    {quizError && <div className={styles.errorMessage}>{quizError}</div>}
                    <button
                      className={`button button--primary ${styles.startQuizButton}`}
                      onClick={generateQuiz}
                    >
                      Generate Quiz
                    </button>
                  </div>
                )}

                {quizState === 'loading' && (
                  <div className={styles.loadingSpinner}>
                    <div className={styles.spinner}></div>
                    <div className={styles.loadingText}>Generating quiz...</div>
                  </div>
                )}

                {quizState === 'questions' && quizQuestions.length > 0 && (
                  <div className={styles.quizQuestion}>
                    <div className={styles.questionNumber}>
                      Question {currentQuestionIndex + 1} of {quizQuestions.length}
                    </div>
                    <div className={styles.questionText}>
                      {quizQuestions[currentQuestionIndex].question}
                    </div>
                    <div className={styles.quizOptions}>
                      {quizQuestions[currentQuestionIndex].options.map((option, idx) => (
                        <div
                          key={idx}
                          className={`${styles.quizOption} ${
                            selectedAnswers[currentQuestionIndex] === option ? styles.selected : ''
                          }`}
                          onClick={() => {
                            const newAnswers = [...selectedAnswers];
                            newAnswers[currentQuestionIndex] = option;
                            setSelectedAnswers(newAnswers);
                          }}
                        >
                          <div className={styles.quizOptionRadio}></div>
                          <div className={styles.quizOptionText}>{option}</div>
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {quizState === 'results' && quizResult && (
                  <div className={styles.quizResults}>
                    <div className={styles.quizResultsIcon}>
                      {quizResult.percentage >= 80 ? '🎉' : quizResult.percentage >= 60 ? '👍' : '📚'}
                    </div>
                    <div className={styles.quizScore}>
                      {quizResult.score}/{quizResult.total}
                    </div>
                    <div className={styles.quizScoreLabel}>
                      {Math.round(quizResult.percentage)}% -{' '}
                      {quizResult.percentage >= 80
                        ? 'Excellent!'
                        : quizResult.percentage >= 60
                        ? 'Good job!'
                        : 'Keep practicing!'}
                    </div>
                    <div className={styles.quizFeedback}>
                      {quizResult.feedback.map((item, idx) => (
                        <div
                          key={idx}
                          className={`${styles.feedbackItem} ${
                            item.startsWith('Q') && item.includes('Correct')
                              ? styles.feedbackCorrect
                              : styles.feedbackIncorrect
                          }`}
                        >
                          {item}
                        </div>
                      ))}
                    </div>
                    <button
                      className="button button--primary"
                      onClick={resetQuiz}
                      style={{ marginTop: '16px' }}
                    >
                      Try Again
                    </button>
                  </div>
                )}
              </div>

              {/* Quiz Navigation */}
              {quizState === 'questions' && (
                <div className={styles.quizNavigation}>
                  <button
                    className="button button--secondary"
                    onClick={() => setCurrentQuestionIndex((prev) => Math.max(0, prev - 1))}
                    disabled={currentQuestionIndex === 0}
                  >
                    Previous
                  </button>
                  {currentQuestionIndex < quizQuestions.length - 1 ? (
                    <button
                      className="button button--primary"
                      onClick={() => setCurrentQuestionIndex((prev) => prev + 1)}
                      disabled={!selectedAnswers[currentQuestionIndex]}
                    >
                      Next
                    </button>
                  ) : (
                    <button
                      className="button button--primary"
                      onClick={submitQuiz}
                      disabled={selectedAnswers.some((a) => !a)}
                    >
                      Submit
                    </button>
                  )}
                </div>
              )}
            </>
          )}
        </div>
      )}
    </>
  );
}
