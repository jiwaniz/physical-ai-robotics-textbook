import React, { useState, useRef, useEffect } from 'react';
import { useLanguage, Language } from '../LanguageContext';
import styles from './LanguageSwitcher.module.css';

const languages: { code: Language; label: string; nativeLabel: string }[] = [
  { code: 'en', label: 'English', nativeLabel: 'English' },
  { code: 'ur', label: 'Urdu', nativeLabel: 'اردو' },
];

export default function LanguageSwitcher() {
  const { language, setLanguage } = useLanguage();
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    }
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const currentLang = languages.find((l) => l.code === language) || languages[0];

  const handleSelect = (code: Language) => {
    setLanguage(code);
    setIsOpen(false);
  };

  return (
    <div className={styles.container} ref={dropdownRef}>
      <button
        className={styles.trigger}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="listbox"
        aria-label="Select language"
      >
        <span className={styles.globe}>🌐</span>
        <span className={styles.label}>{currentLang.nativeLabel}</span>
        <span className={styles.chevron}>{isOpen ? '▲' : '▼'}</span>
      </button>

      {isOpen && (
        <ul className={styles.dropdown} role="listbox">
          {languages.map((lang) => (
            <li
              key={lang.code}
              role="option"
              aria-selected={language === lang.code}
              className={`${styles.option} ${language === lang.code ? styles.selected : ''} ${lang.code === 'ur' ? styles.urduOption : ''}`}
              onClick={() => handleSelect(lang.code)}
            >
              <span className={styles.optionLabel}>{lang.label}</span>
              <span className={lang.code === 'ur' ? styles.urduText : ''}>
                {lang.nativeLabel}
              </span>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
