# ✅ Ready to Run! Complete Step-by-Step Guide

## ✅ What's Already Done:

1. ✅ All 19 code files created (complete auth system)
2. ✅ Database URL configured (Neon PostgreSQL)
3. ✅ Secret key generated and configured
4. ✅ Python virtual environment created
5. ✅ All 60+ packages installed successfully

---

## 🚀 Run These Commands Now:

### Step 1: Create Database Tables

Open your terminal and run:

```bash
cd "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/backend"

./venv/bin/alembic upgrade head
```

**Expected output:**
```
INFO  [alembic.runtime.migration] Running upgrade  -> 001_initial_auth, Create users and user_profiles tables
```

**If you see an error**, it might be a database connection issue. Check `/backend/.env` file.

---

### Step 2: Start Backend Server

In the SAME terminal:

```bash
./venv/bin/uvicorn src.main:app --reload --port 8000
```

**Expected output:**
```
INFO:     Uvicorn running on http://127.0.0.1:8000
INFO:     Application started in development mode
```

**KEEP THIS TERMINAL OPEN!** The backend server is now running.

---

### Step 3: Start Frontend (New Terminal Window)

Open a **NEW terminal** window and run:

```bash
cd "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/frontend"

npm install
```

Wait for npm packages to install (1-2 minutes), then:

```bash
npm start
```

**Expected output:**
```
Compiled successfully!
You can now view the app in the browser.
Local: http://localhost:3000
```

Your browser should automatically open to http://localhost:3000

---

### Step 4: Test Authentication! 🎉

1. **Navigate to:** http://localhost:3000/signup

2. **Create an account:**
   - Name: `Test User`
   - Email: `test@example.com`
   - Password: `Test123!` (must have uppercase, lowercase, number)

3. **Click "Sign Up"**

4. **You'll be redirected to onboarding page**
   - Select your software experience level
   - Select your hardware experience level
   - Check topics you're familiar with
   - Click "Complete Onboarding" (or "Skip for Now")

5. **Success!** You're now authenticated and can browse the textbook!

---

## 🧪 Test Other Features:

### Sign Out and Sign In Again

1. Go to: http://localhost:3000/signin
2. Enter your email and password
3. Click "Sign In"
4. You're authenticated!

### Check Your Profile

1. Make a GET request to view your profile:
```bash
curl http://localhost:8000/api/users/profile \
  --cookie "access_token=YOUR_TOKEN"
```

Or use the browser dev tools to see the cookie.

---

## 🐛 Troubleshooting:

### Backend won't start - "Module not found"
```bash
cd backend
./venv/bin/pip install -r requirements.txt
```

### Backend won't start - "Database connection error"
Check that `/backend/.env` has:
```env
DATABASE_URL=postgresql+asyncpg://neondb_owner:npg_02YZkbpsUgVA@ep-fancy-mode-a74ifw8r-pooler.ap-southeast-2.aws.neon.tech/neondb?ssl=require
```

### Frontend won't start - "Port 3000 already in use"
```bash
# Kill the process on port 3000
lsof -ti:3000 | xargs kill -9
# Or use a different port
PORT=3001 npm start
```

### Migrations fail - "alembic: command not found"
```bash
cd backend
./venv/bin/pip install alembic
```

### Can't access localhost:3000
Make sure:
1. Backend is running (terminal 1)
2. Frontend is running (terminal 2)
3. No firewall blocking ports 3000 or 8000

---

## 📊 API Endpoints Available:

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/auth/signup` | Create new account |
| POST | `/api/auth/signin` | Sign in |
| POST | `/api/auth/signout` | Sign out |
| GET | `/api/auth/me` | Get current user |
| POST | `/api/users/profile` | Create/update profile |
| GET | `/api/users/profile` | Get user profile |

### Test with curl:

```bash
# Signup
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test123!","name":"Test User"}' \
  -c cookies.txt -v

# Signin
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test123!"}' \
  -c cookies.txt -v

# Get current user
curl http://localhost:8000/api/auth/me -b cookies.txt

# Create profile
curl -X POST http://localhost:8000/api/users/profile \
  -H "Content-Type: application/json" \
  -d '{"software_level":"intermediate","hardware_level":"beginner","topics":["python","ml"]}' \
  -b cookies.txt
```

---

## 🎯 You're All Set!

Just run the 4 steps above and your authentication system will be live!

**Everything is ready - all code is written, configured, and installed.** 🚀
