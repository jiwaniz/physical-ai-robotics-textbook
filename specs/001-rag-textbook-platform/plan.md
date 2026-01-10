# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-rag-textbook-platform` | **Date**: 2026-01-05 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-textbook-platform/spec.md`

**Related Artifacts**:
- Comprehensive architectural plan: `/root/.claude/plans/iterative-sprouting-minsky.md`
- Constitution: `.specify/memory/constitution.md` v1.0.0
- Feature specification: `specs/001-rag-textbook-platform/spec.md`

## Summary

Building an educational platform combining a Docusaurus-based static textbook with an intelligent RAG-powered chatbot for the Physical AI & Humanoid Robotics course. The platform provides personalized learning experiences through user profiling (software/hardware background), context-aware assistance via OpenAI-powered semantic search, and adaptive content delivery.

**Core Features**:
- **P1**: Static textbook content (Docusaurus on GitHub Pages)
- **P2**: User authentication and onboarding (Better-auth + Neon Postgres)
- **P3**: RAG chatbot with semantic search (OpenAI + Qdrant + FastAPI)
- **P4**: Text selection for contextual help
- **P5**: Personalized chapter explanations
- **P6**: Chat history persistence

**Technical Approach**:
- Frontend: Docusaurus 3.x static site generator with custom React components
- Backend: FastAPI async Python API with RAG orchestration
- Databases: Neon Serverless Postgres (users, chat) + Qdrant Cloud (vectors)
- AI: OpenAI text-embedding-3-small for embeddings, GPT-4 for completions
- Deployment: GitHub Pages (frontend) + Render (backend) with CI/CD automation

## Technical Context

**Language/Version**:
- Backend: Python 3.11+
- Frontend: Node.js 18+, React 18+

**Primary Dependencies**:
- Backend: FastAPI 0.115+, Better-auth v1.x, OpenAI SDK, SQLAlchemy, Qdrant client
- Frontend: Docusaurus 3.x, Better-auth React SDK, React hooks

**Storage**:
- Neon Serverless Postgres (user accounts, onboarding profiles, chat conversations, chat messages, feedback, analytics)
- Qdrant Cloud Free Tier (1536-dim vector embeddings with metadata)

**Testing**:
- Backend: pytest with 80%+ coverage, integration tests for RAG pipeline
- Frontend: Jest + React Testing Library, E2E with Playwright for critical flows
- Manual: Accessibility testing (WCAG 2.1 AA), cross-browser testing

**Target Platform**:
- Web application (responsive design, 320px minimum width)
- Browsers: Chrome/Firefox/Safari (latest 2 versions)
- Deployment: GitHub Pages (static frontend) + Render (backend API)

**Project Type**: Web application (separate frontend + backend)

**Performance Goals**:
- Frontend: First Contentful Paint (FCP) <1.5s, Largest Contentful Paint (LCP) <2.5s
- Backend: API p95 latency <500ms, chat first token <1s (streaming)
- Search: Qdrant vector search <100ms
- Database: Query latency <50ms

**Constraints**:
- Monthly operational costs <$100 (for 100 active users)
- Must operate within free tiers: GitHub Pages, Render (95 hours/month), Neon (0.5GB), Qdrant (1GB vectors)
- OpenAI API costs optimized via caching (60%+ cache hit rate target)
- JavaScript bundle <200KB gzipped
- 99.5% uptime target

**Scale/Scope**:
- MVP: 100 active users
- Content: 13 weeks × 4 modules = ~52 chapters
- Chat: ~500 queries/day (5 per active user)
- Vector database: ~10,000 content chunks (512 tokens each)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle I: Educational Excellence**
- ✅ FR-002: Each chapter includes learning outcomes, explanations, code examples, troubleshooting
- ✅ Assumption: All code examples tested on Ubuntu 22.04 + ROS 2 Humble before publication
- ✅ FR-001-006: Content organized by modules, searchable, fully responsive, accessible

**Principle II: Universal Accessibility (Personalization)**
- ✅ FR-014-018: Onboarding captures software (Python, AI/ML, ROS) and hardware (Linux, GPU) backgrounds
- ✅ FR-033-037: Personalization adapts chatbot responses and content paths based on user profile
- ✅ FR-006: Keyboard navigation with visible focus indicators (WCAG 2.1 AA)
- ✅ FR-005: Mobile responsive (320px minimum width)

**Principle III: RAG-Powered Support (Intelligent Assistance)**
- ✅ FR-019-028: Semantic search, source citations (module/week/section), rate limiting (10/min), fallback responses
- ✅ FR-021: All chatbot responses cite textbook sources for verification
- ✅ FR-023: Rate limiting prevents abuse, protects against API cost overruns
- ✅ FR-024-025: Fallback mechanisms + user feedback collection for quality monitoring

**Principle IV: Hands-On Learning (Reproducibility)**
- ✅ Assumption: Version pinning (ROS 2 Humble, NVIDIA Isaac 2023.1.1, Ubuntu 22.04)
- ✅ Assumption: Code tested on clean Ubuntu install ("works on my machine" not acceptable)
- ✅ Assessments have clear rubrics and starter code (from spec)
- ✅ Docker containers / VM images recommended for complex setups (from assumptions)

**Principle V: Performance First (Speed & Cost Optimization)**
- ✅ FR-047-051: Performance requirements (FCP <1.5s, API <500ms, chat <1s, caching, graceful degradation)
- ✅ SC-010-017: Success criteria for performance and cost (LCP <2.5s, 99.5% uptime, <$100/month, 60% cache hit)
- ✅ FR-050: Response caching to minimize duplicate OpenAI API calls
- ✅ Embedding generation uses batch API (50% discount per research)

**Principle VI: Security & Privacy (Data Protection)**
- ✅ FR-008: bcrypt password hashing (minimum 10 rounds)
- ✅ FR-009-010: Secure session management with httpOnly cookies, 7-day expiry with sliding renewal
- ✅ FR-052-058: HTTPS enforced, CORS whitelist, input validation (Pydantic), no third-party tracking without consent
- ✅ FR-013: GDPR compliance (users can export/delete data)
- ✅ FR-055-057: API keys in environment variables, security event logging, explicit consent for data collection

**Result: ✅ All principles satisfied. No constitution violations. Proceed to Phase 0.**

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-textbook-platform/
├── spec.md              # Feature specification (user stories, requirements, success criteria)
├── plan.md              # This file - implementation plan
├── research.md          # Technology decisions and best practices
├── data-model.md        # Database schemas and entity relationships
├── quickstart.md        # Setup guide for local development
├── contracts/           # API specifications (OpenAPI/REST)
│   ├── auth.yaml        # Better-Auth endpoints
│   ├── users.yaml       # User management
│   ├── chat.yaml        # RAG chatbot
│   └── personalization.yaml  # Adaptive content
├── checklists/
│   └── requirements.md  # Specification quality checklist (✅ PASSED)
└── tasks.md             # Implementation tasks (created by /sp.tasks - NOT YET CREATED)
```

### Source Code (repository root)

**Selected Structure: Web application (frontend + backend)**

```text
frontend/                         # Docusaurus static site
├── docs/                         # Markdown content
│   ├── 00-introduction/          # Weeks 1-2
│   ├── 01-ros2/                  # Module 1: Weeks 3-5
│   ├── 02-simulation/            # Module 2: Weeks 6-7
│   ├── 03-isaac/                 # Module 3: Weeks 8-10
│   ├── 04-vla/                   # Module 4: Weeks 11-13
│   └── assessments/              # Assessments
├── src/
│   ├── components/
│   │   ├── ChatbotWidget/        # Floating button + slide-out panel
│   │   ├── PersonalizeButton/    # Chapter personalization trigger
│   │   ├── OnboardingModal/      # Multi-step onboarding flow
│   │   └── AuthButtons/          # Login/Signup UI
│   ├── hooks/
│   │   ├── useAuth.ts            # Better-Auth client wrapper
│   │   ├── useChat.ts            # Chat API integration
│   │   └── useTextSelection.ts   # Text selection handler
│   ├── plugins/
│   │   ├── chatbot-integration/  # Docusaurus plugin
│   │   └── text-selection/       # Selection handler plugin
│   └── pages/                    # Custom pages (landing, dashboard)
├── static/img/                   # Images and diagrams
├── docusaurus.config.js          # Main configuration
├── sidebars.js                   # Navigation structure
└── package.json

backend/                          # FastAPI application
├── src/
│   ├── main.py                   # FastAPI entry point, CORS, middleware
│   ├── auth/
│   │   ├── config.py             # Better-Auth setup
│   │   ├── routes.py             # Auth endpoints
│   │   └── middleware.py         # Session validation
│   ├── chat/
│   │   ├── routes.py             # Chat endpoints
│   │   ├── service.py            # RAG orchestration
│   │   └── models.py             # Pydantic models
│   ├── users/
│   │   ├── routes.py             # User management
│   │   ├── service.py            # Business logic
│   │   └── models.py             # User models
│   ├── personalization/
│   │   ├── routes.py             # Personalization endpoints
│   │   ├── service.py            # Prompt building logic
│   │   └── models.py
│   ├── rag/
│   │   ├── embeddings.py         # OpenAI embeddings client
│   │   ├── vector_store.py       # Qdrant client wrapper
│   │   ├── retriever.py          # Search & ranking
│   │   └── agents.py             # OpenAI Agents SDK integration
│   ├── database/
│   │   ├── connection.py         # Neon Postgres connection
│   │   ├── models.py             # SQLAlchemy models
│   │   └── migrations/           # Alembic migrations
│   └── core/
│       ├── config.py             # Settings (Pydantic BaseSettings)
│       ├── exceptions.py         # Custom exceptions
│       └── dependencies.py       # FastAPI dependencies
├── tests/
│   ├── test_auth.py
│   ├── test_chat.py
│   ├── test_rag.py
│   └── test_personalization.py
├── requirements.txt
├── requirements-dev.txt
└── pytest.ini

scripts/
├── embed_content.py              # Batch embedding generation (CI/CD)
├── convert_gdocs.py              # Google Docs → Markdown (optional)
└── setup_databases.py            # Initialize Neon + Qdrant

.github/workflows/
├── deploy-frontend.yml           # Build Docusaurus + deploy to GitHub Pages
├── deploy-backend.yml            # Run tests + deploy to Render
└── embed-content.yml             # Auto-embed on content changes

.specify/                         # SDD framework (existing)
├── memory/constitution.md        # v1.0.0 (6 principles)
├── templates/
└── scripts/

specs/001-rag-textbook-platform/  # This feature's planning docs
history/
├── prompts/                      # PHRs for this feature
│   ├── constitution/
│   └── 001-rag-textbook-platform/
└── adr/                          # Architectural Decision Records
    ├── 001-docusaurus-choice.md
    ├── 002-better-auth.md
    ├── 003-monolithic-api.md
    ├── 004-openai-agents-sdk.md
    └── 005-ssr-vs-csr.md

.env.example                      # Environment variables template
README.md
CLAUDE.md                         # Project rules (SDD workflow)
```

**Structure Decision**: Web application with separate frontend and backend directories. Frontend uses Docusaurus for static site generation with custom React components. Backend is a standalone FastAPI application with modular structure (auth, chat, users, rag, personalization). This separation allows independent scaling and deployment to different platforms (GitHub Pages vs Render).

## Complexity Tracking

**No constitution violations to justify.** All architectural decisions align with the 6 principles. The chosen technology stack (Docusaurus, FastAPI, Better-Auth, OpenAI, Neon, Qdrant) represents industry best practices for educational platforms with AI-powered features, operating within the constraints (cost, performance, security, accessibility).

## Implementation Phases

**Total Timeline**: 10-12 weeks for MVP

### Phase 0: Project Setup & Infrastructure (Week 1)
**Prerequisites**: None
**Deliverables**: Repository structure, CI/CD pipelines, databases provisioned

**Tasks**:
1. Create monorepo structure (frontend/, backend/, scripts/)
2. Initialize Docusaurus project with TypeScript
3. Initialize FastAPI project with poetry/pip
4. Configure linting/formatting (ESLint, Prettier, Black, isort)
5. Set up pre-commit hooks
6. Create .env.example with all required variables
7. Configure Render deployment for backend
8. Configure GitHub Pages for frontend
9. Set up Neon database with dev/staging/prod branches
10. Initialize Qdrant collections for dev/staging/prod
11. Create GitHub Actions workflows (deploy-frontend.yml, deploy-backend.yml, embed-content.yml)

**Acceptance**: `npm run dev` starts Docusaurus locally, `uvicorn src.main:app --reload` starts FastAPI, GitHub Actions pass, can connect to Neon and Qdrant from local environment.

---

### Phase 1: Authentication & User Management (Week 2)
**Prerequisites**: Phase 0 complete
**Deliverables**: Better-Auth integrated, signup/signin functional, sessions working

**Tasks**:
1. Install and configure Better-Auth in FastAPI
2. Create database migrations for users + sessions tables
3. Implement auth endpoints (/signup, /signin, /signout, /session)
4. Configure CORS for cross-origin auth (GitHub Pages origin)
5. Install Better-Auth client SDK in Docusaurus
6. Create useAuth() hook wrapper
7. Build AuthButtons component (Navbar integration)
8. Build login/signup modal forms
9. Implement protected route wrapper
10. Add session persistence and refresh logic
11. Write integration tests for auth flow

**Acceptance**: User can signup, signin, session persists across page refresh, protected API endpoints return 401 for unauthenticated, auth state syncs between Docusaurus and FastAPI.

---

### Phase 2: Content Development & Basic Site (Weeks 2-4)
**Prerequisites**: Phase 0 complete (can run parallel with Phase 1)
**Deliverables**: Textbook content in markdown, Docusaurus site deployed

**Tasks**:
1. Extract outline structure from Google Doc
2. Develop/expand content from outline into full chapters (Weeks 1-5 priority)
3. Create markdown files organized by modules
4. Set up Docusaurus navigation (sidebars.js)
5. Add images and diagrams to static/img/
6. Customize Docusaurus theme (colors, fonts, logo)
7. Swizzle Navbar and Footer components
8. Add search plugin (Algolia DocSearch or local)
9. Configure docusaurus.config.js metadata
10. Test responsive design (mobile, tablet, desktop)
11. Deploy to GitHub Pages
12. Manual content review for accuracy

**Acceptance**: Weeks 1-5 content accessible via navigation, images render, search works, site is mobile-responsive, site loads in <3 seconds on 4G.

---

### Phase 3: Onboarding Flow (Week 3)
**Prerequisites**: Phase 1 complete
**Deliverables**: Onboarding modal, profile storage

**Tasks**:
1. Create database migration for onboarding_responses table
2. Build OnboardingModal component (multi-step)
3. Implement step 1: Software background questionnaire
4. Implement step 2: Hardware background questionnaire
5. Implement step 3: Learning goals selection
6. Create POST /api/users/me/onboarding endpoint
7. Create GET /api/users/me/onboarding endpoint
8. Trigger modal on first login
9. Add "skip for now" option
10. Store completion status in user preferences
11. Write tests for onboarding flow

**Acceptance**: Modal appears on first login, user can complete all steps, onboarding data saved to database, user can revisit/update from dashboard, modal doesn't re-appear after completion.

---

### Phase 4: RAG System - Content Embedding (Week 4)
**Prerequisites**: Phase 2 complete (content exists)
**Deliverables**: Content vectorized in Qdrant, embedding pipeline automated

**Tasks**:
1. Write content chunking logic (semantic splitting, 512 tokens)
2. Implement embedding generation with OpenAI API
3. Create metadata extraction (module, week, topic, difficulty, hardware, code language)
4. Write Qdrant upsert logic
5. Create embed_content.py script
6. Test embedding quality with sample queries
7. Create GitHub Action for auto-embedding on content changes
8. Implement basic search endpoint (POST /api/chat/search)
9. Add error handling and retry logic
10. Monitor embedding costs and optimize chunking
11. Write tests for embedding pipeline

**Acceptance**: All book content embedded in Qdrant, semantic search returns relevant results, metadata filters work, embedding costs <$5 for full book, re-embedding triggered automatically on content updates.

---

### Phase 5: RAG System - Chat Integration (Week 5)
**Prerequisites**: Phase 4 complete (embeddings exist)
**Deliverables**: Chatbot API functional, responses working

**Tasks**:
1. Install OpenAI Agents SDK / ChatKit SDK
2. Implement retrieval logic (Qdrant search, top-k=5)
3. Build context construction (retrieved chunks + user query)
4. Implement chat completion with OpenAI
5. Create POST /api/chat/query endpoint
6. Add conversation management (conversation_id)
7. Implement chat history persistence (database)
8. Create GET /api/chat/history endpoint
9. Add streaming response support (SSE)
10. Implement rate limiting (10 queries/min) and abuse prevention
11. Write integration tests for chat flow

**Acceptance**: Chat API returns relevant answers from book content, responses cite sources (module + week), chat history persists across sessions, handles multi-turn conversations, rate limiting works.

---

### Phase 6: Chatbot UI Widget (Week 6)
**Prerequisites**: Phase 5 complete (Chat API working)
**Deliverables**: Chatbot widget in Docusaurus

**Tasks**:
1. Create ChatbotWidget component structure
2. Build FloatingButton component
3. Build ChatPanel slide-out component
4. Implement ChatMessages display (scrollable history)
5. Build ChatInput component with auto-resize
6. Add typing indicators and loading states
7. Implement error handling UI (retry button)
8. Add "clear conversation" functionality
9. Integrate with useAuth (show login prompt if not authenticated)
10. Add accessibility features (keyboard nav, ARIA labels)
11. Style with Docusaurus theme variables
12. Test on mobile devices

**Acceptance**: Chatbot widget loads on all pages, floating button opens/closes panel smoothly, messages display correctly, user can send messages and see responses, widget is mobile-responsive, accessible via keyboard.

---

### Phase 7: Text Selection Feature (Weeks 6-7)
**Prerequisites**: Phase 6 complete (Chatbot UI working)
**Deliverables**: Text selection handler, context-aware queries

**Tasks**:
1. Create SelectionHandler component
2. Implement mouseup listener for text selection
3. Add minimum selection length validation (>10 chars)
4. Dispatch custom event with selected text
5. Update ChatbotWidget to listen for selection events
6. Add ContextBadge component to show selected text
7. Modify ChatInput to include selected_text in API call
8. Update backend to prioritize selected text in retrieval
9. Add "clear selection" button in chatbot
10. Test selection across different content types (text, code blocks)

**Acceptance**: Selecting text (>10 chars) opens chatbot automatically, selected text appears in chatbot as context, queries reference selected text in responses, user can clear context, works with code blocks.

---

### Phase 8: Personalization System (Weeks 7-8)
**Prerequisites**: Phase 3 + Phase 5 complete (onboarding + chat working)
**Deliverables**: Personalization button, adaptive responses

**Tasks**:
1. Create PersonalizeButton component
2. Add button to DocItem layout (chapter start)
3. Implement personalization service in backend
4. Build prompt template based on user profile
5. Create GET /api/personalization/recommendations endpoint
6. Integrate onboarding data into RAG prompts
7. Add difficulty level filtering in Qdrant queries
8. Implement proactive explanations (triggered by button)
9. Create user dashboard to view/edit preferences
10. A/B test personalization strategies (track metrics)
11. Write tests for personalization logic

**Acceptance**: Personalize button appears at chapter start, clicking opens chatbot with personalized explanation, responses adapt based on user background (beginner vs advanced), user can update preferences, personalization improves relevance (measured by feedback ratings).

---

### Phase 9: Optimization & Polish (Weeks 8-9)
**Prerequisites**: All previous phases complete
**Deliverables**: Performance optimized, monitoring setup

**Tasks**:
1. Implement response caching (Redis or in-memory)
2. Add request debouncing and throttling
3. Optimize Qdrant queries (tune retrieval parameters)
4. Add error boundaries in React components
5. Implement retry logic with exponential backoff
6. Set up monitoring (Sentry for errors, PostHog for analytics)
7. Create feedback UI (thumbs up/down for chat responses)
8. Implement POST /api/chat/feedback endpoint
9. Add loading skeletons and optimistic UI updates
10. Write user documentation (README, FAQ)
11. Perform load testing (simulate 100 concurrent users)
12. Security audit (dependency scanning, CORS review)
13. Accessibility audit (WCAG 2.1 AA compliance)

**Acceptance**: API response time p95 <500ms, frontend bundle <200KB gzipped, error rate <0.1%, all critical user paths accessible, monitoring dashboards functional, user feedback tracked.

---

### Phase 10: Launch & Iteration (Week 10+)
**Prerequisites**: Phase 9 complete
**Deliverables**: Production deployment, user testing, iterations

**Tasks**:
1. Conduct final QA testing
2. Prepare launch announcement
3. Deploy to production (main branch)
4. Invite beta users (5-10 students)
5. Collect feedback via surveys + in-app feedback
6. Monitor analytics for drop-off points
7. Fix critical bugs within 24 hours
8. Prioritize feature requests
9. Iterate on personalization algorithms
10. Expand content (new chapters, case studies)

**Acceptance**: Site publicly accessible, 10+ users onboarded successfully, <3 critical bugs reported, user satisfaction score >4/5, content engagement metrics collected.

---

## Key Architectural Decisions

*See `research.md` for detailed rationale and alternatives considered*

1. **Docusaurus vs Next.js**: Purpose-built for documentation, smaller bundle, native GitHub Pages support
2. **Better-Auth Strategy**: Hybrid (server in FastAPI + client SDK in Docusaurus) for secure sessions with static sites
3. **Content Embedding Pipeline**: Pre-build in CI/CD (not on-demand) for predictable costs and no cold starts
4. **Chatbot UI/UX**: Floating button + slide-out panel (Intercom-style) for familiar, non-intrusive UX
5. **Personalization Strategy**: Same core content + dynamic chatbot responses to avoid content duplication
6. **Deployment**: Multi-environment CI/CD (dev/staging/production) for safe testing

## Risk Mitigation

*See `spec.md` lines 363-408 for comprehensive risk analysis*

**Top Technical Risks**:
1. OpenAI API costs exceed budget → Caching, rate limiting, hard spending limits
2. RAG retrieval quality poor → A/B test chunking strategies, tune parameters, collect user feedback
3. Free tier limits exceeded → Monitor usage proactively, budget for upgrades

**Top UX Risks**:
1. Onboarding too long (users abandon) → User testing, allow skipping, limit to 3 steps max
2. Chatbot perceived as unhelpful → Collect feedback early, iterate on prompts, show example queries

## Critical Files for Implementation

1. `backend/src/main.py` - FastAPI entry point, CORS config, middleware
2. `backend/src/rag/agents.py` - RAG orchestration with OpenAI Agents SDK
3. `frontend/docusaurus.config.js` - Docusaurus config, site structure
4. `frontend/src/components/ChatbotWidget/index.tsx` - Chatbot UI (floating button + panel)
5. `backend/src/database/models.py` - SQLAlchemy models for all tables

## Next Steps

1. Review this plan for approval
2. Create ADRs for major architectural decisions (Docusaurus, Better-Auth, OpenAI Agents SDK, Monolithic API, SSR vs CSR)
3. Run `/sp.tasks` to generate detailed implementation tasks from this plan
4. Begin Phase 0 (Project Setup & Infrastructure)

---

**Plan Status**: ✅ Ready for implementation
**Constitution Compliance**: ✅ All 6 principles satisfied
**Phase 0 Prerequisites**: ✅ Research complete (see research.md), no blocking unknowns
