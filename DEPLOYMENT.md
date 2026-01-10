# Deployment Guide

This guide covers deploying the Physical AI & Humanoid Robotics Textbook Platform.

## Prerequisites

Before deploying, ensure you have:

1. **GitHub Account**: For GitHub Pages (frontend) deployment
2. **Render Account**: For backend API deployment (free tier available)
3. **Neon Account**: For Serverless Postgres database (free tier available)
4. **Qdrant Cloud Account**: For vector database (free tier available)
5. **OpenAI API Key**: For embeddings and chat completions

---

## Frontend Deployment (GitHub Pages)

### Step 1: Configure GitHub Pages Settings

1. Go to your repository on GitHub
2. Navigate to **Settings** → **Pages**
3. Under "Build and deployment":
   - Source: **GitHub Actions**
4. Save the settings

### Step 2: Update Docusaurus Configuration

Edit `frontend/docusaurus.config.js`:

```javascript
url: 'https://<your-username>.github.io',
baseUrl: '/<repository-name>/',
organizationName: '<your-username>',
projectName: '<repository-name>',
```

### Step 3: Deploy

Push to the `main` branch:

```bash
git add .
git commit -m "Configure GitHub Pages deployment"
git push origin main
```

The GitHub Action `.github/workflows/deploy-frontend.yml` will automatically build and deploy.

### Step 4: Verify Deployment

Visit: `https://<your-username>.github.io/<repository-name>/`

---

## Backend Deployment (Render)

### Step 1: Set Up External Services

#### Neon Postgres Database

1. Go to [neon.tech](https://neon.tech)
2. Create a new project
3. Create branches: `main` (production), `dev`, `staging`
4. Copy the connection string for `main` branch
   - Format: `postgresql://user:password@host.neon.tech/dbname`

#### Qdrant Cloud

1. Go to [qdrant.io](https://qdrant.io)
2. Create a new cluster (free tier: 1GB)
3. Create a collection named `book_content`
   - Vector size: 1536 (OpenAI text-embedding-3-small)
   - Distance: Cosine
4. Copy the cluster URL and API key

### Step 2: Deploy to Render

#### Option A: Using render.yaml (Recommended)

1. Go to [render.com](https://render.com)
2. Sign in with GitHub
3. Click **New** → **Blueprint**
4. Connect your repository
5. Render will automatically detect `render.yaml`

#### Option B: Manual Setup

1. Go to [render.com](https://render.com)
2. Click **New** → **Web Service**
3. Connect your GitHub repository
4. Configure:
   - **Name**: physical-ai-textbook-api
   - **Environment**: Python 3
   - **Region**: Oregon (or closest to you)
   - **Branch**: main
   - **Build Command**: `pip install -r backend/requirements.txt`
   - **Start Command**: `uvicorn backend.src.main:app --host 0.0.0.0 --port $PORT`
   - **Plan**: Free

### Step 3: Configure Environment Variables

In Render dashboard, add environment variables:

**Required:**
- `DATABASE_URL`: `<your-neon-connection-string>`
- `NEON_DATABASE_URL`: `<your-neon-connection-string>`
- `QDRANT_URL`: `<your-qdrant-cluster-url>`
- `QDRANT_API_KEY`: `<your-qdrant-api-key>`
- `OPENAI_API_KEY`: `<your-openai-api-key>`
- `BETTER_AUTH_SECRET`: `<generate-256-bit-key>` (use: `openssl rand -base64 32`)

**Optional:**
- `CORS_ORIGINS`: `https://<your-username>.github.io` (update after GitHub Pages deployment)
- `ENVIRONMENT`: `production`
- `DEBUG`: `false`
- `LOG_LEVEL`: `INFO`

### Step 4: Deploy

Render will automatically deploy when you push to `main`:

```bash
git push origin main
```

### Step 5: Run Database Migrations

After first deployment, run migrations via Render Shell:

```bash
cd backend
alembic upgrade head
```

### Step 6: Update Frontend CORS

Update `frontend/src/` to use your Render backend URL:

```typescript
const API_URL = 'https://your-app.onrender.com';
```

Update Render environment variable:
- `CORS_ORIGINS`: Add your GitHub Pages URL

---

## Content Embedding Deployment

### Step 1: Add GitHub Secrets

Go to repository **Settings** → **Secrets and variables** → **Actions**

Add secrets:
- `OPENAI_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `QDRANT_COLLECTION` (optional, defaults to `book_content`)

### Step 2: Trigger Embedding Generation

#### Automatic (on content changes):

When you push markdown files to `frontend/docs/`, the workflow runs automatically.

#### Manual trigger:

1. Go to **Actions** tab on GitHub
2. Select **Generate Content Embeddings** workflow
3. Click **Run workflow**
4. Check "Force regenerate all embeddings" if needed

### Step 3: Verify Embeddings

Check Qdrant Cloud dashboard:
- Collection: `book_content`
- Verify vector count matches number of content chunks

---

## Post-Deployment Checklist

### Frontend
- [ ] Site loads at GitHub Pages URL
- [ ] Navigation works (sidebar, breadcrumbs)
- [ ] Search functionality works
- [ ] Content displays correctly on mobile
- [ ] Accessibility score ≥90 (Lighthouse)

### Backend
- [ ] Health check responds: `https://your-app.onrender.com/health`
- [ ] API docs accessible: `https://your-app.onrender.com/docs`
- [ ] CORS configured (GitHub Pages origin allowed)
- [ ] Database migrations applied
- [ ] Environment variables set

### Integration
- [ ] Frontend can call backend API
- [ ] Embeddings generated in Qdrant
- [ ] OpenAI API calls work (test with chat endpoint)

---

## Monitoring

### Frontend (GitHub Pages)

- **Build logs**: Actions tab → Deploy Frontend workflow
- **Status**: Settings → Pages

### Backend (Render)

- **Logs**: Dashboard → Logs tab
- **Metrics**: Dashboard → Metrics tab
- **Health**: `https://your-app.onrender.com/health`

### Embeddings

- **Qdrant Dashboard**: Check vector count and collection status
- **GitHub Actions**: Actions tab → Generate Content Embeddings workflow

---

## Troubleshooting

### Frontend not deploying
- Check GitHub Actions logs
- Verify `package.json` scripts are correct
- Ensure `docusaurus.config.js` URL/baseUrl are correct

### Backend errors
- Check Render logs for errors
- Verify all environment variables are set
- Test database connection
- Check Render free tier limits (750 hours/month)

### CORS errors
- Update `CORS_ORIGINS` in Render to include GitHub Pages URL
- Redeploy backend after changing CORS settings

### Embeddings not generating
- Check GitHub Actions logs
- Verify OpenAI API key is valid
- Check Qdrant connection
- Ensure markdown files are valid

---

## Cost Estimates (Free Tier Limits)

| Service | Free Tier | Limit |
|---------|-----------|-------|
| GitHub Pages | Free | Public repos |
| Render | Free | 750 hours/month, sleeps after 15min inactive |
| Neon Postgres | Free | 0.5GB storage, 1 project |
| Qdrant Cloud | Free | 1GB vectors |
| OpenAI | Pay-as-you-go | ~$20-50/month (100 users) |

**Total**: $20-50/month for 100 active users

---

## Scaling Beyond Free Tier

When you exceed free tier limits:

1. **Render**: Upgrade to Starter ($7/month) for always-on service
2. **Neon**: Upgrade to Pro ($19/month) for more storage
3. **Qdrant**: Upgrade to paid plan ($25/month) for more vectors
4. **OpenAI**: Implement aggressive caching to reduce API calls

Estimated cost for 500 users: ~$100/month
