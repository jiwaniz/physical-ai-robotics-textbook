---
id: 0005
title: i18n wrap auth components
stage: green
date: 2026-01-31
surface: agent
model: claude-opus-4-5-20251101
feature: 002-urdu-textbook-portal
branch: 002-urdu-textbook-portal
user: Zahra
command: direct prompt
labels: ["i18n", "translate", "auth", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/SigninForm.tsx
  - frontend/src/components/SignupForm.tsx
  - frontend/src/components/EmailVerificationBanner.tsx
  - frontend/src/components/NavbarUserMenu.tsx
tests:
  - none
---

## Prompt

You need to add Docusaurus i18n support to 4 auth component files by wrapping hardcoded English strings with `<Translate>` and `translate()`. Read each file, make the edits, and track the translation keys you create.

Base path: "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/frontend"

## Rules:
1. Add `import Translate, { translate } from '@docusaurus/Translate';` to each file
2. For JSX text content: `<Translate id="key">English text</Translate>`
3. For attributes (placeholder, title, aria-label): `translate({ message: 'English', id: 'key' })`
4. For state strings (setError, etc.): `translate({ message: 'English', id: 'key' })`
5. Key convention: `component.<componentName>.<element>`
6. DO NOT change any logic, styling, or structure — only wrap strings
7. Keep English text as children/message so English locale works without JSON changes

## Files to edit:
1. src/components/SigninForm.tsx
2. src/components/SignupForm.tsx
3. src/components/EmailVerificationBanner.tsx
4. src/components/NavbarUserMenu.tsx

## Response snapshot

Wrapped all hardcoded English strings in 4 auth component files with Docusaurus `<Translate>` and `translate()`. Created 35 translation keys total: 11 in SigninForm, 13 in SignupForm, 6 in EmailVerificationBanner, 5 in NavbarUserMenu. No logic, styling, or structure was changed. All English text preserved as children/message for default locale fallback.

## Outcome

- Impact: All auth UI strings are now i18n-ready for Urdu translation
- Tests: none
- Files: 4 modified
- Next prompts: Create Urdu translation JSON with all 35 keys
- Reflection: Straightforward string wrapping; key naming convention kept consistent

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
