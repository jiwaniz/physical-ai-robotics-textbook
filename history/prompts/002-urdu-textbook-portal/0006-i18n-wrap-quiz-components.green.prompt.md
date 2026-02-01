---
id: 0006
title: i18n Wrap Quiz Components
stage: green
date: 2026-01-31
surface: agent
model: claude-opus-4-5-20251101
feature: 002-urdu-textbook-portal
branch: 002-urdu-textbook-portal
user: human
command: direct-prompt
labels: ["i18n", "quiz", "translate", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/quiz/QuizPlayer.tsx
  - frontend/src/components/quiz/QuizResults.tsx
  - frontend/src/components/quiz/QuizQuestion.tsx
  - frontend/src/components/quiz/QuizProgress.tsx
  - frontend/src/components/quiz/CodeCompletion.tsx
  - frontend/src/components/quiz/ShortAnswer.tsx
tests:
  - none (string-wrapping only, no logic changes)
---

## Prompt

You need to add Docusaurus i18n support to quiz component files by wrapping hardcoded English strings with `<Translate>` and `translate()`. Read each file, make the edits, and track the translation keys you create.

Base path: "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/frontend"

## Rules:
1. Add `import Translate, { translate } from '@docusaurus/Translate';` to each file
2. For JSX text content: `<Translate id="key">English text</Translate>`
3. For attributes (placeholder, title, aria-label): `translate({ message: 'English', id: 'key' })`
4. For state strings (setError, etc.): `translate({ message: 'English', id: 'key' })`
5. Key convention: `component.quiz.<subComponent>.<element>` (e.g., `component.quiz.player.submitButton`)
6. DO NOT change any logic, styling, or structure — only wrap strings
7. Keep English text as children/message so English locale works without JSON changes
8. For `window.confirm()` strings, use `translate()` inside the confirm call

## Files to edit:

### 1. src/components/quiz/QuizPlayer.tsx — wrap loading, error, navigation buttons, confirm dialogs
### 2. src/components/quiz/QuizResults.tsx — wrap headings, score labels, status badges, buttons
### 3. src/components/quiz/QuizQuestion.tsx — wrap question type labels, point labels
### 4. src/components/quiz/QuizProgress.tsx — wrap progress text
### 5. src/components/quiz/CodeCompletion.tsx — wrap hint toggle button text
### 6. src/components/quiz/ShortAnswer.tsx — wrap placeholder text, character count label

After editing all files, output a list of all translation keys you created with their English defaults.

## Response snapshot

Wrapped 38 translation keys across 6 quiz component files using Docusaurus `<Translate>` (JSX) and `translate()` (attributes/state/confirm). No logic, styling, or structural changes made. English text preserved as defaults.

## Outcome

- Impact: All quiz UI strings are now i18n-ready for Urdu and other locales
- Tests: No tests needed; string-wrapping only with no logic changes
- Files: 6 files modified (QuizPlayer, QuizResults, QuizQuestion, QuizProgress, CodeCompletion, ShortAnswer)
- Next prompts: Add Urdu translations to i18n JSON files for these 38 keys
- Reflection: Straightforward mechanical task; parallel file reads + writes kept it efficient

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
