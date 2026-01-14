# 🚀 Quick Start Guide

## Setup and Run Backend (Automated)

I've created a script that will do everything for you automatically!

### Step 1: Run the Setup Script

Open your terminal and run this **ONE** command:

```bash
bash "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/setup-and-run.sh"
```

This script will:
1. ✅ Install all Python dependencies
2. ✅ Create database tables (run migrations)
3. ✅ Start the backend server at http://localhost:8000

---

### Step 2: Start Frontend (In a New Terminal)

Open a **NEW terminal window** and run:

```bash
cd "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/frontend"
npm install
npm start
```

This will start the frontend at http://localhost:3000

---

### Step 3: Test Authentication

1. Open your browser to: **http://localhost:3000/signup**
2. Create an account:
   - Name: Your Name
   - Email: test@example.com
   - Password: Test123! (must have uppercase, lowercase, and number)
3. Complete the onboarding form (or skip)
4. You're authenticated! 🎉

---

## 🐛 Troubleshooting

### If the script fails:

**Check your .env file** has the correct values:
- Database URL: `postgresql+asyncpg://...` (starts with postgresql+asyncpg)
- Secret Key: Should be a long random string

**Check Python version:**
```bash
python3 --version
```
Should be Python 3.8+ (you have 3.14.0 ✓)

**Manually install dependencies:**
```bash
cd "/mnt/e/Zahra/PGD Data Sciences with AI/Agentic AI/Hackathon I - Physical AI & Humanoid Robotics Textbook/backend"
pip3 install -r requirements.txt
```

---

## 📋 What's Been Set Up

✅ Backend `.env` configured with:
- Neon database connection
- JWT secret key
- CORS settings

✅ Frontend `.env` configured with:
- Backend API URL

✅ All code files created:
- Database models (User, UserProfile)
- Auth routes (signup, signin, signout)
- Profile routes (create/get profile)
- Frontend forms (SignupForm, SigninForm, OnboardingForm)
- Frontend pages (signup, signin, onboarding)

---

## 🎯 That's It!

Just run the setup script and you're ready to go! 🚀
