# Tasks: Unified English/Urdu Textbook Portal

**Input**: Design documents from `/specs/002-urdu-textbook-portal/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/profile-api.yaml ✅, quickstart.md ✅

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1–US4)

---

## Phase 1: Backend — Language Preference Column

**Purpose**: Add `preferred_language` to the existing UserProfile model so the backend can persist language choice.

**⚠️ CRITICAL**: Frontend persistence (US2) depends on this phase.

- [x] T001 [US2] Add `preferred_language` column to `UserProfile` model in `backend/src/database/models.py` — `String(5)`, NOT NULL, default `'en'`, CHECK constraint `IN ('en','ur')`
- [x] T002 [US2] Create Alembic migration `alembic revision --autogenerate -m "add preferred_language to user_profiles"` in `backend/src/database/migrations/versions/`
- [ ] T003 [US2] Run migration `alembic upgrade head`
- [x] T004 [P] [US2] Add `preferred_language` field to `UserProfileRequest` and `UserProfileResponse` schemas in `backend/src/users/schemas.py` — optional on request (default `'en'`), required on response; validate `Literal['en','ur']`
- [x] T005 [US2] Verify existing `/api/users/profile` POST and GET endpoints in `backend/src/users/routes.py` handle the new field (no new endpoints needed)

**Checkpoint**: `POST /api/users/profile { "preferred_language": "ur" }` returns 200 with the field persisted. `GET /api/users/profile` includes `preferred_language` in response.

---

## Phase 2: User Story 1 — Switch Language to Urdu (Priority: P1) 🎯 MVP

**Goal**: Student can switch locale to Urdu via navbar dropdown; page renders RTL with Nastaliq font and translated prose; code blocks stay English LTR.

**Independent Test**: Navigate to any chapter, select "اردو" from locale dropdown, verify RTL layout + Nastaliq font + Urdu prose + LTR code blocks.

### RTL & Nastaliq CSS

- [x] T006 [P] [US1] Add Urdu/RTL CSS rules to `frontend/src/css/custom.css`:
  - `html[lang='ur']` / `[dir='rtl']` selector for `font-family: 'Noto Nastaliq Urdu', serif`
  - Override Infima CSS variables for Nastaliq font in RTL context
  - RTL overrides for sidebar, pagination, breadcrumbs, TOC, admonitions, lists
  - Code block LTR preservation: `[dir='rtl'] pre, [dir='rtl'] code { direction: ltr; text-align: left; }`
  - Navbar stays LTR: `[dir='rtl'] .navbar { direction: ltr; }`

### Urdu Translation Files (20 files)

- [x] T007 [US1] Create directory structure `frontend/i18n/ur/docusaurus-plugin-content-docs/current/` mirroring `frontend/docs/`
- [x] T008 [P] [US1] Translate `intro.md` to Urdu → `frontend/i18n/ur/docusaurus-plugin-content-docs/current/intro.md`
- [x] T009 [P] [US1] Translate `00-introduction/index.md` and `week-01.md`, `week-02.md` → Urdu i18n directory
- [x] T010 [P] [US1] Translate `01-ros2/index.md`, `week-03.md`, `week-04.md`, `week-05.md` → Urdu i18n directory
- [x] T011 [P] [US1] Translate `02-simulation/index.md`, `week-06.md`, `week-07.md` → Urdu i18n directory
- [x] T012 [P] [US1] Translate `03-isaac/index.md`, `week-08.md`, `week-09.md`, `week-10.md` → Urdu i18n directory
- [x] T013 [P] [US1] Translate `04-vla/index.md`, `week-11.md`, `week-12.md`, `week-13.md` → Urdu i18n directory
- [x] T014 [P] [US1] Translate `assessments/index.md` → Urdu i18n directory

### UI String Translations

- [x] T015 [US1] Run `npx docusaurus write-translations --locale ur` to generate UI string JSON files in `frontend/i18n/ur/`
- [x] T016 [US1] Translate generated JSON files (sidebar labels, navbar items, footer, theme strings) in `frontend/i18n/ur/`

**Checkpoint**: `npm run build -- --locale ur` succeeds. Visiting `/ur/intro` shows Urdu text, RTL layout, Nastaliq font, LTR code blocks.

---

## Phase 3: User Story 2 — Persist Language Preference (Priority: P1)

**Goal**: Returning students see the portal in their previously chosen language without re-selecting.

**Independent Test**: Switch to Urdu, close browser, reopen site — it loads in Urdu automatically.

**Depends on**: Phase 1 (backend column) complete.

- [x] T017 [US2] Update `AuthContext.tsx` in `frontend/src/components/AuthContext.tsx` to:
  - Fetch `preferred_language` from user profile on auth state change
  - Expose `preferredLanguage` and `setPreferredLanguage` in context
  - On `setPreferredLanguage`, POST to `/api/users/profile` for authenticated users, write to `localStorage` key `preferred-language` for all users
- [x] T018 [US2] Update `Root.tsx` in `frontend/src/theme/Root.tsx` to:
  - On mount, read preferred language from context (authenticated) or `localStorage` (anonymous)
  - If current URL locale differs from preference, redirect to correct locale path (`/` vs `/ur/`)
  - Skip redirect if user explicitly navigated via locale dropdown (check referrer or flag)
- [x] T019 [US2] On locale dropdown change (Docusaurus handles navigation), save the new locale to Supabase (authenticated) or `localStorage` (anonymous) — hook into Docusaurus locale change event or use `useEffect` watching `useDocusaurusContext().i18n.currentLocale`

**Checkpoint**: Authenticated user switches to Urdu → closes browser → reopens → lands on `/ur/`. Unauthenticated user same flow via localStorage.

---

## Phase 4: User Story 3 — Urdu Content for All Chapters (Priority: P2)

**Goal**: Full Urdu translation coverage across all 4 modules (13 weeks) + intro + assessments.

**Independent Test**: Visit each chapter landing page and at least one weekly page per module in Urdu locale — all show Urdu content.

> **Note**: Translation files are created in Phase 2 (T008–T014). This phase is a quality/completeness verification.

- [x] T020 [US3] Review and validate all 20 Urdu translation files for completeness — ensure no untranslated English paragraphs remain, technical terms are transliterated with English in parentheses, code blocks are preserved
- [x] T021 [US3] Verify sidebar labels and category names appear in Urdu by checking `frontend/i18n/ur/docusaurus-plugin-content-docs/current/` `_category_.json` files (create if missing for each subdirectory)
- [x] T022 [US3] Build full Urdu locale: `npm run build -- --locale ur` — verify zero warnings about missing translations

**Checkpoint**: Full Urdu build succeeds with zero translation warnings. All 20 pages render correctly in Urdu.

---

## Phase 5: User Story 4 — Admin Updates Translations (Priority: P3)

**Goal**: Translations can be updated without a site rebuild.

> **Note**: Per research.md R5, this is deferred — static Markdown files are the initial approach. This phase documents the future work for completeness.

- [ ] T023 [US4] **DEFERRED** — Design Supabase-backed translation storage (table schema, admin API endpoint, frontend fetch-and-render logic)
- [ ] T024 [US4] **DEFERRED** — Implement fallback: when no Urdu translation exists for a page, show English content with a "translation pending" banner

**Checkpoint**: Deferred to future iteration. Static files serve launch needs.

---

## Phase 6: Polish & Cross-Cutting

- [x] T025 [P] Verify Nastaliq font loads with `font-display: swap` — no FOIT on slow connections
- [x] T026 [P] Test locale switch performance < 1 second (SC-001)
- [x] T027 [P] Test Urdu page load within 500ms of English (SC-007)
- [x] T028 Run `frontend/npm run build` for both locales — verify zero errors
- [x] T029 Run quickstart.md validation (end-to-end walkthrough)

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Backend)  ──→  Phase 3 (Persistence/US2)
                    ╲
Phase 2 (US1/MVP)  ──→  Phase 4 (US3 - validation)
                    ╲
                     ──→  Phase 6 (Polish)

Phase 5 (US4) = DEFERRED
```

### Parallel Opportunities

- **Phase 1 + Phase 2**: Can run in parallel (backend vs frontend work)
- **T006–T016**: All translation and CSS tasks are parallelizable (different files)
- **T008–T014**: All translation file tasks are fully parallel
- **Phase 3**: Requires Phase 1 complete (needs backend endpoint)
- **Phase 4**: Requires Phase 2 complete (needs translation files to exist)
- **Phase 6**: Requires Phases 2 + 3 complete

### Recommended Execution (Solo Developer)

1. **Start Phase 1 + Phase 2 in parallel** (T001–T005 backend, T006–T016 frontend)
2. **Phase 3** after Phase 1 backend is done (T017–T019)
3. **Phase 4** quality check after all translations exist (T020–T022)
4. **Phase 6** final polish and validation (T025–T029)
5. **Phase 5** deferred — document for future sprint

---

## Task Count Summary

| Phase | Tasks | Status |
|-------|-------|--------|
| Phase 1: Backend | T001–T005 (5) | 4/5 done ✅ (T003 = deploy step) |
| Phase 2: US1 MVP | T006–T016 (11) | 11/11 done ✅ |
| Phase 3: US2 Persistence | T017–T019 (3) | 3/3 done ✅ |
| Phase 4: US3 Full Coverage | T020–T022 (3) | 3/3 done ✅ |
| Phase 5: US4 Admin Updates | T023–T024 (2) | DEFERRED |
| Phase 6: Polish | T025–T029 (5) | 5/5 done ✅ |
| **Total** | **29 tasks** | **26 done, 1 deploy-only (T003), 2 deferred** |
