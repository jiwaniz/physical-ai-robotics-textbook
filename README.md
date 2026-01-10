# Physical AI & Humanoid Robotics Textbook Platform

An interactive educational platform combining a Docusaurus-based textbook with an intelligent RAG-powered chatbot for the Physical AI & Humanoid Robotics course.

## Features

- 📚 **Interactive Textbook**: 13-week curriculum across 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- 🤖 **RAG Chatbot**: AI-powered assistant with semantic search and contextual help
- 👤 **Personalization**: Adaptive content based on user's software/hardware background
- 🔐 **Authentication**: Secure user accounts with Better-auth
- 📝 **Text Selection**: Ask questions about selected text
- 💬 **Chat History**: Persistent conversations across sessions

## Tech Stack

### Frontend
- **Framework**: Docusaurus 3.x (React 18+, TypeScript)
- **Deployment**: GitHub Pages
- **Styling**: Custom CSS with responsive design

### Backend
- **Framework**: FastAPI 0.115+ (Python 3.11+)
- **Authentication**: Better-auth v1.x
- **Databases**:
  - Neon Serverless Postgres (users, chat, onboarding)
  - Qdrant Cloud (vector embeddings)
- **AI**: OpenAI (text-embedding-3-small, GPT-4)
- **Deployment**: Render

## Project Structure

```
├── frontend/               # Docusaurus static site
│   ├── docs/              # Markdown content (4 modules, 13 weeks)
│   ├── src/               # React components, hooks, plugins
│   └── static/            # Images, diagrams
├── backend/               # FastAPI application
│   ├── src/
│   │   ├── auth/          # Better-auth integration
│   │   ├── chat/          # RAG chatbot endpoints
│   │   ├── rag/           # Embeddings, vector store, retrieval
│   │   ├── users/         # User management
│   │   ├── personalization/ # Adaptive prompts
│   │   └── database/      # SQLAlchemy models, migrations
│   └── tests/             # pytest tests
├── scripts/               # Utility scripts (embedding, setup)
├── .github/workflows/     # CI/CD pipelines
└── specs/                 # Design documents
```

## Prerequisites

- **Node.js**: 18+ (for frontend)
- **Python**: 3.11+ (for backend)
- **Accounts**:
  - Neon Serverless Postgres (free tier)
  - Qdrant Cloud (free tier)
  - OpenAI API key
  - GitHub (for deployment)
  - Render (for backend hosting)

## Local Development Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd "Hackathon I - Physical AI & Humanoid Robotics Textbook"
```

### 2. Environment Configuration

Copy `.env.example` to `.env` and fill in your credentials:

```bash
cp .env.example .env
# Edit .env with your API keys and database URLs
```

### 3. Frontend Setup

```bash
cd frontend
npm install
npm start  # Starts dev server at http://localhost:3000
```

### 4. Backend Setup

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
pip install -r requirements-dev.txt  # For development tools

# Run database migrations
alembic upgrade head

# Start backend server
uvicorn src.main:app --reload  # Starts at http://localhost:8000
```

### 5. Run Embedding Pipeline (Optional)

```bash
python scripts/embed_content.py
```

## Development Commands

### Frontend

```bash
npm run start      # Start development server
npm run build      # Build for production
npm run serve      # Serve production build locally
npm run lint       # Run ESLint
npm run format     # Format code with Prettier
npm run typecheck  # TypeScript type checking
```

### Backend

```bash
# Formatting
black src/
isort src/

# Testing
pytest                    # Run all tests
pytest --cov             # With coverage report
pytest tests/test_auth.py  # Run specific test file

# Database migrations
alembic revision --autogenerate -m "description"
alembic upgrade head
alembic downgrade -1

# Start server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

## Deployment

### Frontend (GitHub Pages)

Deployed automatically via GitHub Actions on push to `main` branch.

Manual deployment:
```bash
cd frontend
npm run build
npm run deploy
```

### Backend (Render)

Deployed automatically via GitHub Actions on push to `main` branch.

Manual deployment: Configure Render web service with:
- Build Command: `pip install -r backend/requirements.txt`
- Start Command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
- Environment variables from `.env.example`

## Architecture

### RAG Pipeline

1. **Content Chunking**: Markdown content split into 512-token chunks
2. **Embedding Generation**: OpenAI text-embedding-3-small (1536 dimensions)
3. **Vector Storage**: Qdrant Cloud with metadata filtering
4. **Retrieval**: Semantic search (top-k=5, cosine similarity)
5. **Response Generation**: OpenAI GPT-4 with retrieved context + citations

### Authentication Flow

1. User signs up → Better-auth creates account
2. Complete onboarding questionnaire (software/hardware background)
3. Session stored in Neon Postgres (7-day expiry, httpOnly cookies)
4. Protected routes verify session via middleware

### Personalization

- Beginner: Simplified explanations, avoid advanced jargon
- Intermediate: Balanced technical depth
- Advanced: In-depth implementation details, C++ internals
- Hardware-specific: RTX GPU vs Jetson vs cloud alternatives

## Performance Targets

- Frontend FCP: <1.5s
- Frontend LCP: <2.5s
- API p95 latency: <500ms
- Chat first token: <1s
- Bundle size: <200KB gzipped
- Lighthouse score: 90+

## Security

- HTTPS enforced
- CORS whitelist (GitHub Pages origin)
- httpOnly session cookies
- bcrypt password hashing (10 rounds)
- Rate limiting (10 queries/min)
- Input validation (Pydantic)
- GDPR compliance (account deletion)

## Cost Budget

- GitHub Pages: $0
- Render Free Tier: $0 (95 hours/month)
- Neon Free Tier: $0 (0.5GB storage)
- Qdrant Free Tier: $0 (1GB vectors)
- OpenAI API: ~$20-50/month (100 active users)
- **Total**: <$100/month

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines.

## License

Educational use only.

## Support

For issues or questions, open a GitHub issue or contact the development team.
