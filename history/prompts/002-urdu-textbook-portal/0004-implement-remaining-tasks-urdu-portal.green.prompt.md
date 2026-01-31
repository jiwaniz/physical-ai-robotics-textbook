---
id: 0004
title: Implement Remaining Tasks Urdu Portal
stage: green
date: 2026-01-31
surface: agent
model: claude-opus-4-5-20251101
feature: 002-urdu-textbook-portal
branch: 002-urdu-textbook-portal
user: zahra
command: /sp.implement
labels: ["implementation", "urdu", "i18n", "build-fix", "mdx"]
links:
  spec: specs/002-urdu-textbook-portal/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - frontend/i18n/ur/code.json
  - frontend/i18n/ur/docusaurus-theme-classic/navbar.json
  - frontend/i18n/ur/docusaurus-theme-classic/footer.json
  - frontend/i18n/ur/docusaurus-plugin-content-docs/current.json
  - frontend/i18n/ur/docusaurus-plugin-content-docs/current/assessments/index.md
  - frontend/i18n/ur/docusaurus-plugin-content-docs/current/04-vla/week-13.md
  - frontend/i18n/ur/docusaurus-plugin-content-docs/current/00-introduction/index.md
  - frontend/i18n/ur/docusaurus-plugin-content-docs/current/01-ros2/index.md
  - frontend/i18n/ur/docusaurus-plugin-content-docs/current/02-simulation/index.md
  - frontend/i18n/ur/docusaurus-plugin-content-docs/current/03-isaac/index.md
  - specs/002-urdu-textbook-portal/tasks.md
tests:
  - npm run build -- --locale ur (PASS)
  - npm run build -- --locale en (PASS)
---

## Prompt

all remaining task from @specs/002-urdu-textbook-portal/tasks.md

## Response snapshot

Implemented all remaining tasks (T003, T019-T022, T025-T029). Key work: (1) Installed missing @supabase/supabase-js dependency, (2) Fixed MDX compilation errors caused by unescaped `<` characters in Urdu translations (assessments/index.md, 04-vla/week-13.md), (3) Fixed broken markdown links in 4 index.md files (old filenames like week-1-foundations.md → week-01.md), (4) Verified T019 already implemented via LocaleSyncOnChange component, (5) Validated all 20 translation files, (6) Built both en and ur locales successfully. T003 (alembic upgrade head) requires live database — migration file exists. 26/29 tasks done, 1 deploy-only, 2 deferred.

## Outcome

- ✅ Impact: All implementable tasks complete; both locale builds succeed with zero errors
- 🧪 Tests: Urdu build PASS, English build PASS
- 📁 Files: 11 files modified (6 translation fixes, 4 JSON translations, 1 tasks.md update)
- 🔁 Next prompts: Run T003 migration on live DB, commit and create PR
- 🧠 Reflection: MDX's JSX parser treats bare `<` as tag openers — always escape in markdown content

## Evaluation notes (flywheel)

- Failure modes observed: Unescaped `<` in MDX causing build failures; missing npm dependency; stale link filenames in translations
- Graders run and results (PASS/FAIL): Build ur PASS, Build en PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Add pre-build lint for unescaped `<` in i18n markdown files
