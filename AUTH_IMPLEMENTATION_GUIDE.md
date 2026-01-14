# Authentication Implementation Guide

## Overview

This document provides a complete guide for the authentication system implemented for the Physical AI & Humanoid Robotics Textbook platform. The implementation uses **native FastAPI JWT authentication** (not Better Auth, as it's Node.js-only) with an optional user background questionnaire for personalized learning.

## What Was Implemented

### Backend (FastAPI)

1. **Database Models** (`backend/src/database/models.py`)
   - `User` model: email, password_hash, name, is_active
   - `UserProfile` model: user_id, software_level, hardware_level, topics (JSON)

2. **Alembic Migration** (`backend/src/database/migrations/versions/001_create_users_and_profiles.py`)
   - Creates `users` and `user_profiles` tables
   - Establishes foreign key relationships
   - Adds indexes for performance

3. **Authentication Logic**
   - **schemas.py**: Pydantic models for signup/signin requests and responses
   - **utils.py**: Password hashing (bcrypt) and JWT token creation/validation
   - **routes.py**: POST /signup, POST /signin, POST /signout, GET /me

4. **User Profile API** (`backend/src/users/`)
   - **schemas.py**: Profile request/response models with experience level enum
   - **routes.py**: POST /profile (upsert), GET /profile (retrieve)

5. **Auth Dependencies** (`backend/src/core/dependencies.py`)
   - `get_current_user_id_from_cookie()`: Extracts user ID from JWT cookie
   - `require_auth()`: Dependency for protected endpoints

6. **Router Registration** (`backend/src/main.py`)
   - Registered auth and users routers with `/api/auth` and `/api/users` prefixes

### Frontend (Docusaurus + React)

1. **Auth Context** (`frontend/src/components/AuthContext.tsx`)
   - Global auth state management
   - Methods: signup, signin, signout, checkAuth
   - Auto-checks auth on mount

2. **Form Components**
   - **SignupForm.tsx**: Email, password, name fields with validation
   - **SigninForm.tsx**: Email, password fields
   - **OnboardingForm.tsx**: Software/hardware levels + topics checkboxes

3. **Pages**
   - **signup.tsx**: Signup page at `/signup`
   - **signin.tsx**: Signin page at `/signin`
   - **onboarding.tsx**: Protected onboarding page at `/onboarding`

4. **Styling** (`frontend/src/css/auth.css`)
   - Form styling with light/dark theme support
   - Responsive design for mobile devices

## Setup Instructions

### 1. Backend Setup

#### Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

#### Configure Environment Variables

Create a `.env` file in `/backend`:

```env
# Database
DATABASE_URL=postgresql+asyncpg://user:password@host:5432/dbname
NEON_DATABASE_URL=${DATABASE_URL}

# Authentication
BETTER_AUTH_SECRET=your-256-bit-secret-key-here-generate-with-openssl-rand-hex-32
BETTER_AUTH_URL=http://localhost:8000
BETTER_AUTH_TRUST_HOST=true

# CORS
CORS_ORIGINS=http://localhost:3000,https://yourgithubpages.com

# Security
SESSION_COOKIE_SECURE=false  # Set to true in production
SESSION_COOKIE_SAMESITE=lax
SESSION_MAX_AGE_DAYS=7

# Environment
ENVIRONMENT=development
DEBUG=true
LOG_LEVEL=INFO
```

#### Run Database Migrations

```bash
cd backend
alembic upgrade head
```

#### Start Backend Server

```bash
cd backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Frontend Setup

#### Install Dependencies

```bash
cd frontend
npm install
```

#### Configure Environment Variables

Create a `.env` file in `/frontend`:

```env
REACT_APP_API_URL=http://localhost:8000
```

#### Integrate AuthProvider

Update `frontend/src/theme/Root.tsx` (create if doesn't exist):

```tsx
import React from 'react';
import { AuthProvider } from '../components/AuthContext';

export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}
```

#### Import Auth CSS

Update `frontend/src/css/custom.css`:

```css
@import './auth.css';
```

#### Add Navbar Auth Link

Update `frontend/docusaurus.config.js` to add auth links:

```js
module.exports = {
  // ... existing config
  themeConfig: {
    navbar: {
      items: [
        // ... existing items
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
        },
        {
          to: '/signin',
          label: 'Sign In',
          position: 'right',
        },
      ],
    },
  },
};
```

#### Start Frontend Server

```bash
cd frontend
npm start
```

## User Flow

### 1. Signup Flow

1. User navigates to `/signup`
2. Fills out name, email, password
3. Password validated (8+ chars, uppercase, lowercase, digit)
4. POST to `/api/auth/signup`
5. JWT token set in httpOnly cookie
6. Redirected to `/onboarding`

### 2. Onboarding Flow (Optional)

1. User selects software experience level (beginner/intermediate/advanced)
2. User selects hardware experience level (beginner/intermediate/advanced)
3. User checks familiar topics (Python, ML, ROS2, etc.)
4. POST to `/api/users/profile`
5. Profile saved for learning path personalization
6. User can skip this step

### 3. Signin Flow

1. User navigates to `/signin`
2. Enters email and password
3. POST to `/api/auth/signin`
4. JWT token set in httpOnly cookie
5. Redirected to home (`/`)

### 4. Protected Content Access

1. User makes request to protected endpoint
2. `require_auth` dependency extracts JWT from cookie
3. If valid, request proceeds
4. If invalid/missing, returns 401 Unauthorized

## API Endpoints

### Authentication

| Method | Endpoint          | Description          | Auth Required |
|--------|-------------------|----------------------|---------------|
| POST   | `/api/auth/signup` | Create new account   | No            |
| POST   | `/api/auth/signin` | Sign in to account   | No            |
| POST   | `/api/auth/signout`| Sign out user        | No            |
| GET    | `/api/auth/me`     | Get current user     | Yes           |

### User Profile

| Method | Endpoint             | Description              | Auth Required |
|--------|----------------------|--------------------------|---------------|
| POST   | `/api/users/profile` | Create/update profile    | Yes           |
| GET    | `/api/users/profile` | Get current user profile | Yes           |

## Testing

### Backend Testing

Run unit and integration tests:

```bash
cd backend
pytest tests/ --cov
```

### Frontend Testing

Run component tests:

```bash
cd frontend
npm test
```

### Manual Testing

1. **Test Signup**:
   ```bash
   curl -X POST http://localhost:8000/api/auth/signup \
     -H "Content-Type: application/json" \
     -d '{"email":"test@example.com","password":"Test123!","name":"Test User"}' \
     -c cookies.txt -v
   ```

2. **Test Signin**:
   ```bash
   curl -X POST http://localhost:8000/api/auth/signin \
     -H "Content-Type: application/json" \
     -d '{"email":"test@example.com","password":"Test123!"}' \
     -c cookies.txt -v
   ```

3. **Test Protected Endpoint**:
   ```bash
   curl http://localhost:8000/api/auth/me -b cookies.txt
   ```

4. **Test Profile Creation**:
   ```bash
   curl -X POST http://localhost:8000/api/users/profile \
     -H "Content-Type: application/json" \
     -d '{"software_level":"intermediate","hardware_level":"beginner","topics":["python","ml"]}' \
     -b cookies.txt
   ```

## Security Features

- Password hashing with bcrypt (work factor 12)
- JWT tokens stored in httpOnly cookies (prevents XSS)
- Secure cookie flag in production (HTTPS only)
- SameSite=lax cookie policy (CSRF protection)
- Generic error messages (prevents email enumeration)
- Password strength validation (8+ chars, mixed case, numbers)
- 7-day token expiry
- CORS restricted to approved origins

## Database Schema

### users table
```sql
CREATE TABLE users (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    name VARCHAR(255) NOT NULL,
    is_active BOOLEAN DEFAULT TRUE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### user_profiles table
```sql
CREATE TABLE user_profiles (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    user_id INTEGER UNIQUE NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    software_level VARCHAR(20),
    hardware_level VARCHAR(20),
    topics JSON,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

## Next Steps (Optional Enhancements)

1. **Rate Limiting**: Add rate limiting to prevent brute force attacks
2. **Email Verification**: Send verification emails after signup
3. **Password Reset**: Implement forgot password flow
4. **OAuth Providers**: Add Google/GitHub OAuth
5. **MFA**: Implement two-factor authentication
6. **Session Management**: Add user session dashboard
7. **Admin Panel**: Create admin interface for user management

## Troubleshooting

### Issue: CORS Errors

**Solution**: Ensure `CORS_ORIGINS` in `.env` includes your frontend URL (e.g., `http://localhost:3000`)

### Issue: JWT Token Not Set

**Solution**: Verify `credentials: 'include'` is set in all fetch requests

### Issue: Database Migration Fails

**Solution**:
1. Check `DATABASE_URL` is correct
2. Ensure database exists
3. Run `alembic upgrade head` from `/backend` directory

### Issue: Onboarding Redirect Loop

**Solution**: Ensure `AuthProvider` wraps the entire app in `Root.tsx`

## File Structure

```
backend/
├── src/
│   ├── auth/
│   │   ├── routes.py          # Auth endpoints
│   │   ├── schemas.py         # Pydantic models
│   │   └── utils.py           # Password/JWT utilities
│   ├── users/
│   │   ├── routes.py          # Profile endpoints
│   │   └── schemas.py         # Profile models
│   ├── core/
│   │   └── dependencies.py    # Auth dependencies
│   ├── database/
│   │   ├── models.py          # User/UserProfile models
│   │   └── migrations/
│   │       └── versions/
│   │           └── 001_create_users_and_profiles.py
│   └── main.py                # Router registration
└── requirements.txt

frontend/
├── src/
│   ├── components/
│   │   ├── AuthContext.tsx    # Auth state management
│   │   ├── SignupForm.tsx     # Signup form
│   │   ├── SigninForm.tsx     # Signin form
│   │   └── OnboardingForm.tsx # Background questionnaire
│   ├── pages/
│   │   ├── signup.tsx         # Signup page
│   │   ├── signin.tsx         # Signin page
│   │   └── onboarding.tsx     # Onboarding page
│   └── css/
│       └── auth.css           # Form styling
└── package.json
```

## Support

For issues or questions, refer to the FastAPI and Docusaurus documentation:
- [FastAPI Authentication](https://fastapi.tiangolo.com/tutorial/security/)
- [Docusaurus Documentation](https://docusaurus.io/docs)
- [JWT Best Practices](https://tools.ietf.org/html/rfc8725)
