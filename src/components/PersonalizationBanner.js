import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

const PersonalizationBanner = () => {
  const [showBanner, setShowBanner] = useState(false);
  const [userName, setUserName] = useState('');
  const [userLevel, setUserLevel] = useState('beginner');
  const [preferences, setPreferences] = useState({});
  const location = useLocation();

  // Load preferences from localStorage on component mount
  useEffect(() => {
    const savedName = localStorage.getItem('user_name');
    const savedLevel = localStorage.getItem('user_level') || 'beginner';
    const savedPreferences = localStorage.getItem('user_preferences');
    
    if (savedName) setUserName(savedName);
    setUserLevel(savedLevel);
    if (savedPreferences) setPreferences(JSON.parse(savedPreferences));
    
    // Only show banner on initial visit to the site
    const hasVisited = localStorage.getItem('has_visited');
    if (!hasVisited) {
      setShowBanner(true);
      localStorage.setItem('has_visited', 'true');
    }
  }, []);

  const handleSavePreferences = () => {
    localStorage.setItem('user_name', userName);
    localStorage.setItem('user_level', userLevel);
    localStorage.setItem('user_preferences', JSON.stringify(preferences));
    setShowBanner(false);
  };

  // Don't show on certain pages
  if (location.pathname.includes('/page/')) {
    return null;
  }

  if (!showBanner) {
    return (
      <div className="personalization-summary">
        <p>
          Welcome back, <strong>{userName || 'Learner'}</strong>! 
          {userName && ` Your preferred learning level is ${userLevel}.`}
        </p>
      </div>
    );
  }

  return (
    <div className="personalization-banner">
      <div className="personalization-content">
        <h3>Personalize Your Learning Experience</h3>
        <p>Help us tailor the content to your needs:</p>
        
        <div className="personalization-form">
          <div className="form-group">
            <label htmlFor="userName">Name:</label>
            <input
              id="userName"
              type="text"
              value={userName}
              onChange={(e) => setUserName(e.target.value)}
              placeholder="Your name"
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="userLevel">Experience Level:</label>
            <select
              id="userLevel"
              value={userLevel}
              onChange={(e) => setUserLevel(e.target.value)}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>
          
          <div className="form-group">
            <label>
              <input
                type="checkbox"
                checked={preferences.showCodeExamples}
                onChange={(e) => setPreferences({...preferences, showCodeExamples: e.target.checked})}
              />
              Show detailed code examples
            </label>
          </div>
          
          <div className="form-group">
            <label>
              <input
                type="checkbox"
                checked={preferences.showQuizzes}
                onChange={(e) => setPreferences({...preferences, showQuizzes: e.target.checked})}
              />
              Include quizzes and assessments
            </label>
          </div>
          
          <div className="form-group">
            <label>
              <input
                type="checkbox"
                checked={preferences.paceFast}
                onChange={(e) => setPreferences({...preferences, paceFast: e.target.checked})}
              />
              Prefer fast-paced learning
            </label>
          </div>
        </div>
        
        <div className="banner-actions">
          <button onClick={handleSavePreferences} className="primary-button">
            Save Preferences
          </button>
          <button onClick={() => setShowBanner(false)} className="secondary-button">
            Skip for now
          </button>
        </div>
      </div>
    </div>
  );
};

export default PersonalizationBanner;