# Quickstart: Unified English/Urdu Textbook Portal

**Feature**: 002-urdu-textbook-portal

## Prerequisites

- Node.js 18+ and npm
- Python 3.11+ with pip
- Access to Supabase project (credentials in `backend/.env`)
- PostgreSQL access (Neon or local)

## Implementation Order

### Step 1: Backend — Add Language Preference

1. Add `preferred_language` column to `UserProfile` model in `backend/src/database/models.py`
2. Create migration: `alembic revision --autogenerate -m "add preferred_language to user_profiles"`
3. Run migration: `alembic upgrade head`
4. Update `UserProfileRequest` and `UserProfileResponse` schemas in `backend/src/users/schemas.py`
5. Verify existing `/api/users/profile` POST and GET endpoints handle the new field

### Step 2: Frontend — Urdu Translation Files

1. Create directory: `frontend/i18n/ur/docusaurus-plugin-content-docs/current/`
2. Copy all files from `frontend/docs/` into the i18n directory
3. Translate each Markdown file to Urdu (keep front matter in English, keep code blocks in English)
4. Run `npx docusaurus write-translations --locale ur` to generate UI string translation files
5. Translate generated JSON files in `frontend/i18n/ur/`

### Step 3: Frontend — RTL/Nastaliq CSS

1. Add Urdu-specific CSS rules to `frontend/src/css/custom.css`:
   - `html[lang='ur']` or `[dir='rtl']` selectors for Nastaliq font
   - RTL overrides for sidebar, pagination, admonitions, lists
   - Code block LTR preservation

### Step 4: Frontend — Language Preference Sync

1. Update `AuthContext.tsx` to fetch and expose `preferred_language` from user profile
2. Update `Root.tsx` to check preferred language on mount and redirect to correct locale URL
3. On locale change (Docusaurus handles dropdown), save preference to Supabase (authenticated) or localStorage (anonymous)

### Step 5: Verify

```bash
# Backend
cd backend
alembic upgrade head
uvicorn src.main:app --reload
# Test: POST /api/users/profile with preferred_language: "ur"

# Frontend
cd frontend
npm start
# Test: Switch locale dropdown to اردو, verify RTL + Nastaliq + translated content
npm run build -- --locale ur
# Test: Build succeeds for Urdu locale
```

## Key Files Changed

| File | Change |
|------|--------|
| `backend/src/database/models.py` | Add `preferred_language` column |
| `backend/src/database/migrations/versions/002_*.py` | New migration |
| `backend/src/users/schemas.py` | Add field to request/response |
| `frontend/i18n/ur/**/*.md` | All Urdu translations (20+ files) |
| `frontend/src/css/custom.css` | RTL and Nastaliq styles |
| `frontend/src/components/AuthContext.tsx` | Language preference sync |
| `frontend/src/theme/Root.tsx` | Auto-redirect to preferred locale |
