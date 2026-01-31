# Feature Specification: Unified English/Urdu Textbook Portal

**Feature Branch**: `002-urdu-textbook-portal`
**Created**: 2026-01-31
**Status**: Draft
**Input**: User description: "Unified English/Urdu Textbook Portal with Supabase persistence"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Switch Language to Urdu (Priority: P1)

A student visits the Physical AI & Humanoid Robotics textbook and wants to read course content in Urdu. They use the locale dropdown in the navbar to switch from English to Urdu. The entire page layout switches to right-to-left, headings and body text render in Noto Nastaliq Urdu, and all translatable prose appears in Urdu. Code blocks, technical commands, and code examples remain in English.

**Why this priority**: Core value proposition — the portal is unusable for Urdu-speaking students without language switching.

**Independent Test**: Navigate to any chapter page, switch locale to Urdu via the navbar dropdown, and verify that prose content is displayed in Urdu with RTL layout while code blocks stay LTR English.

**Acceptance Scenarios**:

1. **Given** a student is on any English page, **When** they select "اردو" from the locale dropdown, **Then** the page reloads in Urdu with RTL direction, Nastaliq font, and translated prose.
2. **Given** a student is viewing the Urdu locale, **When** they select "English" from the locale dropdown, **Then** the page returns to LTR English with the default font.
3. **Given** a page contains code blocks, **When** displayed in Urdu locale, **Then** code blocks remain in English with LTR direction and monospace font.

---

### User Story 2 - Persist Language Preference (Priority: P1)

A returning student who previously chose Urdu opens the textbook again. The portal remembers their language preference and automatically loads content in Urdu without requiring them to switch again.

**Why this priority**: Equal to P1 — without persistence, students must re-select their language on every visit, making the feature frustrating to use.

**Independent Test**: Switch language to Urdu, close the browser, reopen the site, and verify it loads in Urdu automatically.

**Acceptance Scenarios**:

1. **Given** an authenticated student has set their language to Urdu, **When** they revisit the portal in a new session, **Then** the portal loads in Urdu by default.
2. **Given** an unauthenticated visitor switches to Urdu, **When** they revisit on the same device, **Then** the portal loads in Urdu (via local storage fallback).
3. **Given** a student changes their language from Urdu to English, **When** they revisit, **Then** the portal loads in English.

---

### User Story 3 - Urdu Content for All Chapters (Priority: P2)

A student navigates through the full 13-week course content — all four modules (ROS 2, Simulation, Isaac, VLA) — and finds Urdu translations available for every chapter and section. The sidebar, table of contents, and navigation elements are also in Urdu.

**Why this priority**: Extends P1 from a single page to the full course; incomplete coverage reduces trust and usability.

**Independent Test**: Visit each chapter landing page and at least one weekly page per module in Urdu locale and verify Urdu content is present.

**Acceptance Scenarios**:

1. **Given** the Urdu locale is active, **When** a student navigates to any chapter (00 through 04), **Then** the chapter index and all weekly pages display Urdu translations.
2. **Given** the Urdu locale is active, **When** a student views the sidebar, **Then** sidebar labels and category names appear in Urdu.
3. **Given** the Urdu locale is active, **When** a student views the intro page, **Then** it displays the fully translated Urdu introduction.

---

### User Story 4 - Admin Updates Translations (Priority: P3)

A course administrator or translator updates Urdu translations for a specific chapter. The updated translation is persisted and immediately available to students without requiring a site rebuild.

**Why this priority**: Supports ongoing content maintenance; static translations work for launch but dynamic updates are needed for long-term sustainability.

**Independent Test**: Update a translation record, reload the corresponding page in Urdu locale, and verify the updated text appears.

**Acceptance Scenarios**:

1. **Given** a translation record exists for a page, **When** an administrator updates the Urdu text, **Then** students see the updated translation on their next page load.
2. **Given** no translation record exists for a newly added page, **When** a student views it in Urdu locale, **Then** the system displays a fallback (English content with a notice that translation is pending).

---

### Edge Cases

- What happens when a student switches locale mid-quiz? The quiz state should be preserved and the quiz content should remain in the language it was started in.
- How does the system handle pages with mixed content (embedded videos, diagrams with English labels)? Non-text media remains unchanged regardless of locale.
- What happens when a translation is incomplete (only some paragraphs translated)? The system shows available Urdu text and falls back to English for untranslated sections, with a visual indicator.
- What happens on very slow connections when loading Nastaliq font? The page renders with a system fallback font first, then re-renders when Nastaliq loads (font-display: swap).
- What happens when Supabase is unreachable? The system falls back to locally stored preference (localStorage) and static translation files.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST support two locales: English (`en`) as default and Urdu (`ur`).
- **FR-002**: System MUST provide a locale dropdown in the navbar allowing users to switch between English and Urdu.
- **FR-003**: System MUST render Urdu content with right-to-left (RTL) text direction.
- **FR-004**: System MUST render Urdu text using the Noto Nastaliq Urdu font family.
- **FR-005**: System MUST preserve code blocks, terminal commands, and inline code in English with LTR direction regardless of active locale.
- **FR-006**: System MUST provide Urdu translations for all 13 weekly pages across 4 modules, the intro page, and assessment pages.
- **FR-007**: System MUST provide Urdu translations for sidebar labels, navigation elements, and category names.
- **FR-008**: System MUST persist each authenticated user's language preference in Supabase, associated with their user profile.
- **FR-009**: System MUST fall back to localStorage for language persistence when the user is not authenticated.
- **FR-010**: System MUST automatically load the user's persisted language preference on subsequent visits.
- **FR-011**: System MUST display English content with a "translation pending" notice when an Urdu translation is unavailable for a page.
- **FR-012**: System MUST store translation content in Supabase to allow updates without site rebuilds.
- **FR-013**: System MUST apply appropriate RTL styling to sidebar, pagination, breadcrumbs, table of contents, admonitions, and lists when Urdu locale is active.
- **FR-014**: System MUST keep the navbar in LTR layout regardless of locale for consistent icon and control placement.

### Key Entities

- **User Profile**: Represents a registered student; extended with a `preferred_language` attribute (`en` or `ur`).
- **Translation Record**: Represents the Urdu translation of a specific page; keyed by page path, contains translated HTML/Markdown content, last updated timestamp, and status (complete, partial, pending).
- **Locale Configuration**: Represents the supported locales and their display properties (label, direction, font family).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can switch between English and Urdu within 1 second via the locale dropdown.
- **SC-002**: 100% of chapter pages (20+ pages) have complete Urdu translations available at launch.
- **SC-003**: A returning authenticated user's language preference is correctly restored on 100% of visits.
- **SC-004**: All Urdu text renders in Nastaliq script with correct RTL layout on modern browsers (Chrome, Firefox, Safari, Edge — latest 2 versions).
- **SC-005**: Code blocks remain legible and correctly formatted in English on Urdu locale pages with zero layout breakage.
- **SC-006**: Language preference persistence survives browser restart for both authenticated (Supabase) and unauthenticated (localStorage) users.
- **SC-007**: Page load time with Urdu locale is within 500ms of English locale load time.

## Assumptions

- The existing Docusaurus i18n infrastructure (already configured with `en` and `ur` locales and `localeDropdown`) will be used as the foundation for static locale switching.
- Supabase is already integrated for authentication and user profiles; the `preferred_language` field will be added to the existing user profile structure.
- Translation content will be initially provided as static Markdown files in the `i18n/ur/` directory and optionally backed by Supabase for dynamic updates (P3).
- The Noto Nastaliq Urdu Google Font is already linked via `headTags` in the Docusaurus config.
- Technical terms (ROS 2, NVIDIA Isaac, Gazebo, VLA, etc.) will be transliterated in Urdu rather than translated, with English in parentheses where helpful.
- The AI chatbot will continue to operate in English regardless of locale (Urdu chatbot support is out of scope).

## Out of Scope

- Translation of the AI chatbot interface or responses into Urdu.
- Support for languages other than English and Urdu.
- Machine translation or AI-powered on-the-fly translation of content.
- Translation of embedded videos, images, or diagram labels.
- Urdu translations for the admin/score tracking interfaces.
