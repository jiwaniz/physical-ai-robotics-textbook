# Research: Unified English/Urdu Textbook Portal

**Feature**: 002-urdu-textbook-portal
**Date**: 2026-01-31

## R1: Docusaurus i18n for Urdu (RTL)

**Decision**: Use Docusaurus built-in i18n with static Markdown files in `i18n/ur/docusaurus-plugin-content-docs/current/`.

**Rationale**: Docusaurus i18n is the official, zero-dependency solution. It supports RTL via `localeConfigs.ur.direction: 'rtl'`, generates separate URL paths per locale (`/ur/...`), and the `localeDropdown` navbar item is already configured. No plugins or custom code needed for basic locale switching.

**Alternatives considered**:
- Custom React translation component (UrduToggle): Rejected — requires runtime API calls, adds latency, fragile DOM manipulation, not SSR-compatible.
- Third-party i18n library (react-i18next): Rejected — Docusaurus has built-in support; adding another library is unnecessary complexity.
- AI-powered on-the-fly translation: Rejected — unreliable for educational content quality, adds API dependency and cost.

## R2: Language Preference Persistence

**Decision**: Add `preferred_language` column (VARCHAR(5), default 'en') to the existing `UserProfile` model. Sync via existing `/api/users/profile` endpoint. Use localStorage fallback for unauthenticated users.

**Rationale**: The `UserProfile` model already stores user preferences (software_level, hardware_level, topics). Adding a language field follows the same pattern. The existing profile upsert endpoint handles create-or-update, so no new endpoints are needed.

**Alternatives considered**:
- Separate language preferences table: Rejected — over-engineering for a single field.
- Supabase user_metadata: Rejected — would bypass the application's profile model and create inconsistency with existing preference storage.
- Cookie-only persistence: Rejected — doesn't survive cross-device usage for authenticated users.

## R3: Noto Nastaliq Urdu Font Loading

**Decision**: Load via Google Fonts using the `headTags` configuration already in `docusaurus.config.js`. Apply with `font-display: swap` (Google Fonts default) to avoid FOIT.

**Rationale**: Google Fonts CDN is globally distributed and cached across sites. The font is already linked in the config. CSS rules apply the font conditionally when `[dir='rtl']` or `html[lang='ur']` is present.

**Alternatives considered**:
- Self-hosting the font: Rejected — adds 2-4MB to the repo, requires manual updates, no caching benefit.
- Using system Urdu fonts: Rejected — inconsistent rendering across OS; Nastaliq is not available on most non-Urdu systems.

## R4: RTL Styling Strategy

**Decision**: Use Docusaurus's built-in RTL support (sets `dir="rtl"` on `<html>` when Urdu locale is active) combined with targeted CSS overrides for components that don't auto-flip.

**Rationale**: Docusaurus automatically applies RTL direction to the document root when a locale with `direction: 'rtl'` is active. Most Infima CSS uses logical properties (e.g., `padding-inline-start`) that auto-flip. Only specific components (code blocks, admonitions, pagination) need manual CSS overrides to maintain correct layout.

**Alternatives considered**:
- CSS-in-JS RTL transform: Rejected — Docusaurus uses CSS modules and Infima; adding a CSS-in-JS layer conflicts.
- Separate RTL stylesheet: Rejected — duplicates maintenance; conditional CSS within custom.css is cleaner.

## R5: Translation Content Strategy

**Decision**: Create static Markdown files in `i18n/ur/docusaurus-plugin-content-docs/current/` mirroring the `docs/` structure. Human-translated content with technical terms transliterated.

**Rationale**: Static files are the Docusaurus standard. They are version-controlled, reviewable, and require zero runtime infrastructure. The `docusaurus write-translations` command can scaffold JSON translation files for UI strings (sidebar labels, navbar items).

**Alternatives considered**:
- Supabase-stored translations fetched at runtime: Deferred to P3 (admin updates story) — adds unnecessary complexity for initial launch.
- AI-generated translations: Rejected — educational content requires human review for accuracy and pedagogical quality.

## R6: Locale Preference Auto-Redirect

**Decision**: On page load, check the user's stored language preference (Supabase profile for authenticated, localStorage for anonymous) and redirect to the correct locale path if it differs from the current URL.

**Rationale**: Docusaurus i18n uses URL paths (`/` for English, `/ur/` for Urdu). Without auto-redirect, returning users always land on English. A small client-side script in `Root.tsx` checks the preference and triggers `window.location` redirect if needed.

**Alternatives considered**:
- Server-side redirect: Not possible — frontend is static GitHub Pages.
- Docusaurus plugin for locale detection: No official plugin exists for Supabase-backed preference.
