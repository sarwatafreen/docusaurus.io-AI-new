import React, { useState } from 'react';

const Quiz = ({ questions }) => {
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [selectedAnswer, setSelectedAnswer] = useState(null);
  const [showResult, setShowResult] = useState(false);
  const [score, setScore] = useState(0);
  const [answers, setAnswers] = useState([]);

  const handleAnswer = (answerIndex) => {
    setSelectedAnswer(answerIndex);
  };

  const handleNextQuestion = () => {
    const newAnswers = [...answers, {
      question: questions[currentQuestion].question,
      selected: selectedAnswer,
      correct: questions[currentQuestion].correctAnswer
    }];
    setAnswers(newAnswers);

    // Check if answer is correct
    if (selectedAnswer === questions[currentQuestion].correctAnswer) {
      setScore(score + 1);
    }

    if (currentQuestion + 1 < questions.length) {
      setCurrentQuestion(currentQuestion + 1);
      setSelectedAnswer(null);
    } else {
      setShowResult(true);
    }
  };

  const restartQuiz = () => {
    setCurrentQuestion(0);
    setSelectedAnswer(null);
    setShowResult(false);
    setScore(0);
    setAnswers([]);
  };

  if (showResult) {
    return (
      <div className="quiz-result">
        <h3>Quiz Result</h3>
        <p>Your score: {score}/{questions.length}</p>
        <p>{score === questions.length ? 'Perfect! 🎉' : score >= questions.length / 2 ? 'Good job! 👍' : 'Keep learning! 📚'}</p>
        <ul>
          {answers.map((answer, index) => (
            <li key={index}>
              <strong>Q{index + 1}:</strong> {answer.question}<br />
              <strong>Your answer:</strong> {answer.selected === answer.correct ? '✓ Correct' : '✗ Incorrect'}
            </li>
          ))}
        </ul>
        <button onClick={restartQuiz} className="restart-button">Restart Quiz</button>
      </div>
    );
  }

  const currentQ = questions[currentQuestion];
  return (
    <div className="quiz-container">
      <div className="progress-bar">
        <span>Question {currentQuestion + 1} of {questions.length}</span>
        <div className="progress">
          <div 
            className="progress-fill" 
            style={{ width: `${((currentQuestion + 1) / questions.length) * 100}%` }}
          ></div>
        </div>
      </div>
      
      <h3 className="quiz-question">{currentQ.question}</h3>
      
      <ul className="quiz-options">
        {currentQ.options.map((option, index) => (
          <li 
            key={index}
            className={`quiz-option ${selectedAnswer === index ? 'selected' : ''}`}
            onClick={() => handleAnswer(index)}
          >
            <input
              type="radio"
              id={`option-${index}`}
              name="quiz-option"
              checked={selectedAnswer === index}
              onChange={() => handleAnswer(index)}
            />
            <label htmlFor={`option-${index}`}>{option}</label>
          </li>
        ))}
      </ul>
      
      {selectedAnswer !== null && (
        <button 
          onClick={handleNextQuestion}
          className="next-button"
        >
          {currentQuestion + 1 === questions.length ? 'Finish Quiz' : 'Next Question'}
        </button>
      )}
    </div>
  );
};

export default Quiz;