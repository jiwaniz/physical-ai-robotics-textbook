# 🚀 Super Simple Start Guide

## What's Ready

✅ All code files created (auth system complete!)
✅ Database URL configured (Neon)
✅ Secret key configured
✅ Virtual environment created
⏳ Python packages installing now (takes 2-3 minutes)

---

## Step-by-Step Instructions

### Step 1: Wait for Installation (or run manually)

**If the installation is still running**, just wait for it to finish.

**OR run it yourself:**

```bash
cd "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/backend"

# Activate virtual environment and install
./venv/bin/pip install -r requirements.txt
```

You'll see: `Successfully installed ...` when done.

---

### Step 2: Create Database Tables

```bash
cd "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/backend"

./venv/bin/alembic upgrade head
```

You should see: `Running upgrade -> 001_initial_auth`

---

### Step 3: Start Backend Server

```bash
cd "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/backend"

./venv/bin/uvicorn src.main:app --reload --port 8000
```

You should see: `Uvicorn running on http://127.0.0.1:8000`

**Keep this terminal open!**

---

### Step 4: Start Frontend (New Terminal)

Open a **NEW terminal** window:

```bash
cd "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/frontend"

npm install
npm start
```

Browser will open at: http://localhost:3000

---

### Step 5: Test Authentication! 🎉

1. Go to: **http://localhost:3000/signup**
2. Create account:
   - Name: Test User
   - Email: test@example.com
   - Password: Test123!
3. Fill onboarding form (or skip)
4. You're authenticated!

---

## Quick Troubleshooting

### "Module not found"
```bash
cd backend
./venv/bin/pip install -r requirements.txt
```

### "Database connection error"
Check `/backend/.env` has your Neon URL starting with `postgresql+asyncpg://`

### "Port already in use"
Change port: `./venv/bin/uvicorn src.main:app --reload --port 8001`

---

## That's It!

Everything is set up. Just run the 5 steps above and you're done! 🚀
