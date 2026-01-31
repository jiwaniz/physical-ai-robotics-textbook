# Data Model: Unified English/Urdu Textbook Portal

**Feature**: 002-urdu-textbook-portal
**Date**: 2026-01-31

## Entity Changes

### UserProfile (existing — modified)

**Location**: `backend/src/database/models.py`

| Field | Type | Constraint | Notes |
|-------|------|-----------|-------|
| id | Integer | PK, auto-increment | Existing |
| supabase_user_id | String(255) | Unique, not null | Existing — links to Supabase Auth |
| software_level | Enum | nullable | Existing |
| hardware_level | Enum | nullable | Existing |
| topics | JSON | nullable | Existing |
| **preferred_language** | **String(5)** | **not null, default 'en'** | **NEW — 'en' or 'ur'** |
| created_at | DateTime | auto | Existing |
| updated_at | DateTime | auto | Existing |

**Validation rules**:
- `preferred_language` MUST be one of: `'en'`, `'ur'`
- Default value: `'en'`
- Non-nullable — every profile has a language preference

### Migration

**File**: `backend/src/database/migrations/versions/002_add_preferred_language.py`

- Add column `preferred_language` VARCHAR(5) NOT NULL DEFAULT 'en' to `user_profiles` table
- No data migration needed — default covers all existing rows

## Schema Changes

### UserProfileRequest (existing — modified)

**Location**: `backend/src/users/schemas.py`

| Field | Type | Required | Notes |
|-------|------|----------|-------|
| software_level | ExperienceLevel | optional | Existing |
| hardware_level | ExperienceLevel | optional | Existing |
| topics | List[str] | optional | Existing |
| **preferred_language** | **str** | **optional, default 'en'** | **NEW** |

**Validation**: `preferred_language` must be in `['en', 'ur']`

### UserProfileResponse (existing — modified)

| Field | Type | Notes |
|-------|------|-------|
| id | int | Existing |
| supabase_user_id | str | Existing |
| software_level | str | Existing |
| hardware_level | str | Existing |
| topics | List[str] | Existing |
| **preferred_language** | **str** | **NEW** |
| created_at | datetime | Existing |
| updated_at | datetime | Existing |

## Frontend State

### localStorage

| Key | Value | Purpose |
|-----|-------|---------|
| `preferred-language` | `'en'` or `'ur'` | Fallback for unauthenticated users; also used as cache for authenticated users to avoid API call on every page load |

### Locale URL Pattern

| Locale | URL Pattern | Example |
|--------|-------------|---------|
| English (default) | `/physical-ai-robotics-textbook/` | `/physical-ai-robotics-textbook/intro` |
| Urdu | `/physical-ai-robotics-textbook/ur/` | `/physical-ai-robotics-textbook/ur/intro` |

## Relationships

```text
Supabase Auth User (1) ──── (1) UserProfile
                                  └── preferred_language: 'en' | 'ur'
```

No new entities. No new relationships. Single column addition to existing model.
