import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';

const API_BASE_URL = (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL) || 'https://jiwaniz-physical-ai-backend.hf.space';

interface OnboardingFormProps {
  allowSkip?: boolean;
}

const OnboardingForm: React.FC<OnboardingFormProps> = ({ allowSkip = true }) => {
  const [softwareLevel, setSoftwareLevel] = useState<string>('');
  const [hardwareLevel, setHardwareLevel] = useState<string>('');
  const [topics, setTopics] = useState<string[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const history = useHistory();
  const homeUrl = useBaseUrl('/');

  const availableTopics = [
    'Python',
    'C++',
    'Machine Learning',
    'Deep Learning',
    'Computer Vision',
    'ROS 2',
    'Robotics',
    'Control Systems',
    'Embedded Systems',
    'Kinematics',
    'Path Planning',
    'SLAM',
  ];

  const handleTopicToggle = (topic: string) => {
    setTopics((prev) =>
      prev.includes(topic) ? prev.filter((t) => t !== topic) : [...prev, topic]
    );
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    if (!softwareLevel || !hardwareLevel) {
      setError('Please select both software and hardware experience levels');
      return;
    }

    setIsSubmitting(true);

    try {
      const response = await fetch(`${API_BASE_URL}/api/users/profile`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          software_level: softwareLevel,
          hardware_level: hardwareLevel,
          topics: topics.map((t) => t.toLowerCase().replace(/\s+/g, '_')),
        }),
        credentials: 'include',
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Failed to save profile');
      }

      // Redirect to home after successful profile creation
      history.push(homeUrl);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to save profile');
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleSkip = () => {
    history.push(homeUrl);
  };

  return (
    <div className="onboarding-form-container">
      <h2>Tell Us About Your Background</h2>
      <p>Help us personalize your learning experience</p>

      <form onSubmit={handleSubmit} className="onboarding-form">
        {error && (
          <div className="alert alert-danger" role="alert">
            {error}
          </div>
        )}

        <div className="form-group">
          <label htmlFor="software-level">Software Development Experience</label>
          <select
            id="software-level"
            className="form-control"
            value={softwareLevel}
            onChange={(e) => setSoftwareLevel(e.target.value)}
            required
          >
            <option value="">Select your experience level</option>
            <option value="beginner">Beginner - New to programming</option>
            <option value="intermediate">Intermediate - Some programming experience</option>
            <option value="advanced">Advanced - Experienced developer</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="hardware-level">Hardware/Robotics Experience</label>
          <select
            id="hardware-level"
            className="form-control"
            value={hardwareLevel}
            onChange={(e) => setHardwareLevel(e.target.value)}
            required
          >
            <option value="">Select your experience level</option>
            <option value="beginner">Beginner - New to hardware/robotics</option>
            <option value="intermediate">
              Intermediate - Some hardware/robotics experience
            </option>
            <option value="advanced">Advanced - Experienced with hardware/robotics</option>
          </select>
        </div>

        <div className="form-group">
          <label>Topics and Technologies You're Familiar With</label>
          <small className="form-text text-muted mb-2">
            Select all that apply (optional)
          </small>
          <div className="topics-grid">
            {availableTopics.map((topic) => (
              <div key={topic} className="form-check">
                <input
                  type="checkbox"
                  className="form-check-input"
                  id={`topic-${topic}`}
                  checked={topics.includes(topic)}
                  onChange={() => handleTopicToggle(topic)}
                />
                <label className="form-check-label" htmlFor={`topic-${topic}`}>
                  {topic}
                </label>
              </div>
            ))}
          </div>
        </div>

        <div className="button-group">
          <button
            type="submit"
            className="button button--primary button--lg"
            disabled={isSubmitting}
          >
            {isSubmitting ? 'Saving...' : 'Complete Onboarding'}
          </button>

          {allowSkip && (
            <button
              type="button"
              className="button button--secondary button--lg"
              onClick={handleSkip}
              disabled={isSubmitting}
            >
              Skip for Now
            </button>
          )}
        </div>
      </form>
    </div>
  );
};

export default OnboardingForm;
