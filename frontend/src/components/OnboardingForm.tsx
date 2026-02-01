import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Translate, { translate } from '@docusaurus/Translate';
import { useAuth } from './AuthContext';

interface OnboardingFormProps {
  allowSkip?: boolean;
}

const OnboardingForm: React.FC<OnboardingFormProps> = ({ allowSkip = true }) => {
  const { accessToken, apiBaseUrl } = useAuth();
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
      setError(translate({ message: 'Please select both software and hardware experience levels', id: 'component.onboardingForm.error.selectBoth' }));
      return;
    }

    setIsSubmitting(true);

    try {
      const headers: HeadersInit = {
        'Content-Type': 'application/json',
      };
      if (accessToken) {
        headers['Authorization'] = `Bearer ${accessToken}`;
      }

      const response = await fetch(`${apiBaseUrl}/api/users/profile`, {
        method: 'POST',
        headers,
        body: JSON.stringify({
          software_level: softwareLevel,
          hardware_level: hardwareLevel,
          topics: topics.map((t) => t.toLowerCase().replace(/\s+/g, '_')),
        }),
        credentials: 'include',
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || translate({ message: 'Failed to save profile', id: 'component.onboardingForm.error.saveFailed' }));
      }

      // Redirect to home after successful profile creation
      history.push(homeUrl);
    } catch (err) {
      setError(err instanceof Error ? err.message : translate({ message: 'Failed to save profile', id: 'component.onboardingForm.error.saveFailed' }));
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleSkip = () => {
    history.push(homeUrl);
  };

  return (
    <div className="onboarding-form-container">
      <h2><Translate id="component.onboardingForm.heading">Tell Us About Your Background</Translate></h2>
      <p><Translate id="component.onboardingForm.subheading">Help us personalize your learning experience</Translate></p>

      <form onSubmit={handleSubmit} className="onboarding-form">
        {error && (
          <div className="alert alert-danger" role="alert">
            {error}
          </div>
        )}

        <div className="form-group">
          <label htmlFor="software-level"><Translate id="component.onboardingForm.softwareLabel">Software Development Experience</Translate></label>
          <select
            id="software-level"
            className="form-control"
            value={softwareLevel}
            onChange={(e) => setSoftwareLevel(e.target.value)}
            required
          >
            <option value="">{translate({ message: 'Select your experience level', id: 'component.onboardingForm.selectLevel' })}</option>
            <option value="beginner">{translate({ message: 'Beginner - New to programming', id: 'component.onboardingForm.software.beginner' })}</option>
            <option value="intermediate">{translate({ message: 'Intermediate - Some programming experience', id: 'component.onboardingForm.software.intermediate' })}</option>
            <option value="advanced">{translate({ message: 'Advanced - Experienced developer', id: 'component.onboardingForm.software.advanced' })}</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="hardware-level"><Translate id="component.onboardingForm.hardwareLabel">Hardware/Robotics Experience</Translate></label>
          <select
            id="hardware-level"
            className="form-control"
            value={hardwareLevel}
            onChange={(e) => setHardwareLevel(e.target.value)}
            required
          >
            <option value="">{translate({ message: 'Select your experience level', id: 'component.onboardingForm.selectLevel' })}</option>
            <option value="beginner">{translate({ message: 'Beginner - New to hardware/robotics', id: 'component.onboardingForm.hardware.beginner' })}</option>
            <option value="intermediate">{translate({ message: 'Intermediate - Some hardware/robotics experience', id: 'component.onboardingForm.hardware.intermediate' })}</option>
            <option value="advanced">{translate({ message: 'Advanced - Experienced with hardware/robotics', id: 'component.onboardingForm.hardware.advanced' })}</option>
          </select>
        </div>

        <div className="form-group">
          <label><Translate id="component.onboardingForm.topicsLabel">Topics and Technologies You're Familiar With</Translate></label>
          <small className="form-text text-muted mb-2">
            <Translate id="component.onboardingForm.topicsHint">Select all that apply (optional)</Translate>
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
            {isSubmitting ? translate({ message: 'Saving...', id: 'component.onboardingForm.saving' }) : translate({ message: 'Complete Onboarding', id: 'component.onboardingForm.submit' })}
          </button>

          {allowSkip && (
            <button
              type="button"
              className="button button--secondary button--lg"
              onClick={handleSkip}
              disabled={isSubmitting}
            >
              <Translate id="component.onboardingForm.skip">Skip for Now</Translate>
            </button>
          )}
        </div>
      </form>
    </div>
  );
};

export default OnboardingForm;
