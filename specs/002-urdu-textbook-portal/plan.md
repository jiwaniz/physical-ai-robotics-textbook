# Implementation Plan: Unified English/Urdu Textbook Portal

**Branch**: `002-urdu-textbook-portal` | **Date**: 2026-01-31 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-urdu-textbook-portal/spec.md`

## Summary

Add bilingual English/Urdu support to the Physical AI & Humanoid Robotics textbook using Docusaurus i18n for static locale switching, Supabase for persisting user language preferences, and static Markdown translation files for Urdu content. The locale dropdown (already configured) triggers page reloads with RTL layout and Nastaliq font for Urdu. Language preference is stored in the existing `UserProfile` model via a new `preferred_language` column, with localStorage fallback for unauthenticated users.

## Technical Context

**Language/Version**: Python 3.11 (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: FastAPI 0.115+, Docusaurus 3.x, Supabase JS SDK, React 18
**Storage**: Neon PostgreSQL (user profiles), Supabase Auth (identity), static Markdown files (translations)
**Testing**: pytest (backend), manual browser testing (frontend i18n)
**Target Platform**: Web (GitHub Pages frontend, Render backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: Locale switch < 1s, Urdu page load within 500ms of English
**Constraints**: Noto Nastaliq Urdu font load via Google Fonts (font-display: swap), no runtime translation API
**Scale/Scope**: 20+ Markdown pages to translate, ~100 active users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| I. Educational Excellence | PASS | Urdu translations expand accessibility; code examples stay English |
| II. Universal Accessibility | PASS | Bilingual support directly serves this principle; RTL/Nastaliq for Urdu readers |
| III. RAG-Powered Support | PASS (N/A) | Chatbot stays English — out of scope per spec |
| IV. Hands-On Learning | PASS | Code blocks preserved in English with LTR; tutorials remain reproducible |
| V. Performance First | PASS | Static Markdown translations add zero runtime cost; font load uses swap |
| VI. Security & Privacy | PASS | Language preference is non-sensitive; stored via existing authenticated profile endpoint |
| VII. Agentic Development | PASS | Incremental changes to existing models; no new services or abstractions needed |
| Tech Stack | PASS | Docusaurus i18n (built-in), FastAPI profile endpoint (existing), Supabase (existing) |

**Pre-Phase 0 Gate: PASS** — No violations. No complexity tracking needed.

## Project Structure

### Documentation (this feature)

```text
specs/002-urdu-textbook-portal/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── profile-api.yaml
└── tasks.md             # Phase 2 output (/sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── database/
│   │   ├── models.py            # Add preferred_language to UserProfile
│   │   └── migrations/versions/ # New migration for column
│   └── users/
│       ├── routes.py            # Existing profile CRUD (schema updated)
│       └── schemas.py           # Add preferred_language field
└── tests/
    └── test_users.py            # Test language preference persistence

frontend/
├── docusaurus.config.js         # Already configured: i18n en/ur, localeDropdown, headTag
├── i18n/
│   └── ur/
│       └── docusaurus-plugin-content-docs/
│           └── current/         # All 20+ Urdu Markdown files
├── src/
│   ├── components/
│   │   └── AuthContext.tsx       # Add language preference sync to Supabase
│   ├── css/
│   │   └── custom.css           # Add Urdu/RTL styles
│   └── theme/
│       └── Root.tsx             # Add locale preference loader
└── docs/                        # Existing English content (unchanged)
```

**Structure Decision**: Existing web application structure (frontend + backend) is preserved. Changes are additive — new column on existing model, new i18n directory, CSS additions. No new services, modules, or abstractions.

## Complexity Tracking

No violations — table intentionally left empty.
