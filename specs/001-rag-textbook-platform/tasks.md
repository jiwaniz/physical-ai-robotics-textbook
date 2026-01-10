---
description: "Task list for Physical AI & Humanoid Robotics Textbook Platform"
---

# Tasks: Physical AI & Humanoid Robotics Textbook Platform

**Input**: Design documents from `/specs/001-rag-textbook-platform/`
**Prerequisites**: plan.md, spec.md, constitution.md v1.0.0

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below follow plan.md structure (frontend/ + backend/ separation)

---

## Phase 1: Setup (Shared Infrastructure) ✅ COMPLETE

**Purpose**: Project initialization and basic structure

- [X] T001 Create monorepo directory structure: frontend/, backend/, scripts/, .github/workflows/
- [X] T002 [P] Initialize Docusaurus project with TypeScript in frontend/
- [X] T003 [P] Initialize FastAPI project with requirements.txt in backend/
- [X] T004 [P] Configure ESLint, Prettier for frontend/
- [X] T005 [P] Configure Black, isort for backend/
- [X] T006 Create .env.example with all required environment variables (OPENAI_API_KEY, NEON_DATABASE_URL, QDRANT_URL, QDRANT_API_KEY, BETTER_AUTH_SECRET)
- [X] T007 [P] Create .gitignore for Python and Node.js projects
- [X] T008 [P] Setup pre-commit hooks for linting and formatting
- [X] T009 Create README.md with project overview and setup instructions
- [X] T010 [P] Create frontend/package.json with Docusaurus 3.x and dependencies
- [X] T011 [P] Create backend/requirements.txt with FastAPI, Better-auth, OpenAI, SQLAlchemy, Qdrant client

---

## Phase 2: Foundational (Blocking Prerequisites) ✅ COMPLETE

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database & Infrastructure Setup

- [X] T012 Provision Neon Serverless Postgres database with dev/staging/prod branches (Manual setup required - see DEPLOYMENT.md)
- [X] T013 Provision Qdrant Cloud Free Tier account and create collections for dev/staging/prod (Manual setup required - see DEPLOYMENT.md)
- [X] T014 Create backend/src/core/config.py with Pydantic BaseSettings for environment configuration
- [X] T015 Create backend/src/database/connection.py with SQLAlchemy engine and session management for Neon Postgres
- [X] T016 Setup Alembic in backend/src/database/migrations/ for database migrations
- [X] T017 Create backend/src/rag/vector_store.py with Qdrant client wrapper and connection management

### Base Models & Error Handling

- [X] T018 [P] Create backend/src/core/exceptions.py with custom exception classes (AuthenticationError, RateLimitError, NotFoundError, ValidationError)
- [X] T019 [P] Create backend/src/core/dependencies.py with FastAPI dependency injection utilities
- [X] T020 Create backend/src/database/models.py with SQLAlchemy base model class

### API Foundation

- [X] T021 Create backend/src/main.py with FastAPI app initialization, CORS configuration for GitHub Pages origin
- [X] T022 Add middleware to backend/src/main.py for request logging, error handling, and request ID tracking
- [X] T023 [P] Create backend/src/auth/middleware.py with session validation middleware
- [X] T024 Setup health check endpoint GET /health in backend/src/main.py

### CI/CD Setup

- [X] T025 Create .github/workflows/deploy-frontend.yml for building Docusaurus and deploying to GitHub Pages
- [X] T026 Create .github/workflows/deploy-backend.yml for running backend tests and deploying to Render
- [X] T027 Create .github/workflows/embed-content.yml for automatic embedding generation on content changes
- [X] T028 Configure Render web service for backend deployment with environment variables
- [X] T029 Configure GitHub Pages settings for frontend deployment

**Checkpoint**: ✅ Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Browse Educational Content (Priority: P1) 🎯 MVP

**Goal**: Deploy static Docusaurus textbook with responsive content, navigation, and search

**Independent Test**: Deploy static site, verify all chapters accessible, navigation works, content displays correctly on desktop and mobile

### Content Structure Setup

- [ ] T030 [P] [US1] Create frontend/docs/00-introduction/ directory with index.md for Weeks 1-2
- [ ] T031 [P] [US1] Create frontend/docs/01-ros2/ directory with index.md for Module 1 (Weeks 3-5)
- [ ] T032 [P] [US1] Create frontend/docs/02-simulation/ directory with index.md for Module 2 (Weeks 6-7)
- [ ] T033 [P] [US1] Create frontend/docs/03-isaac/ directory with index.md for Module 3 (Weeks 8-10)
- [ ] T034 [P] [US1] Create frontend/docs/04-vla/ directory with index.md for Module 4 (Weeks 11-13)
- [ ] T035 [P] [US1] Create frontend/docs/assessments/ directory with assessment content

### Docusaurus Configuration

- [ ] T036 [US1] Configure frontend/docusaurus.config.js with site metadata (title, tagline, url, baseUrl, favicon)
- [ ] T037 [US1] Configure frontend/sidebars.js with module/week navigation hierarchy
- [ ] T038 [US1] Add Algolia DocSearch or local search plugin to frontend/docusaurus.config.js
- [ ] T039 [US1] Customize Docusaurus theme in frontend/docusaurus.config.js (colors, fonts, logo)
- [ ] T040 [US1] Swizzle Navbar component to frontend/src/theme/Navbar/ for custom branding
- [ ] T041 [US1] Swizzle Footer component to frontend/src/theme/Footer/ with copyright and links

### Content Development (Weeks 1-5 Priority)

- [ ] T042 [US1] Develop Week 1 content in frontend/docs/00-introduction/week-01.md with learning outcomes and introductory material
- [ ] T043 [US1] Develop Week 2 content in frontend/docs/00-introduction/week-02.md with prerequisites and setup guides
- [ ] T044 [US1] Develop Week 3 content in frontend/docs/01-ros2/week-03.md (ROS 2 basics)
- [ ] T045 [US1] Develop Week 4 content in frontend/docs/01-ros2/week-04.md (Nodes and Topics)
- [ ] T046 [US1] Develop Week 5 content in frontend/docs/01-ros2/week-05.md (Services and Actions)
- [ ] T047 [P] [US1] Add code examples and diagrams to frontend/static/img/ for Weeks 1-5

### Responsive Design & Accessibility

- [ ] T048 [US1] Test responsive design on mobile (320px width), tablet, and desktop breakpoints
- [ ] T049 [US1] Add keyboard navigation support with visible focus indicators
- [ ] T050 [US1] Run Lighthouse accessibility audit and achieve score of 90+
- [ ] T051 [US1] Test with screen reader (NVDA/JAWS) and fix accessibility issues

### Deployment

- [ ] T052 [US1] Configure GitHub Pages deployment from frontend/ build output
- [ ] T053 [US1] Deploy to GitHub Pages and verify site is publicly accessible
- [ ] T054 [US1] Verify search functionality works on deployed site
- [ ] T055 [US1] Verify site loads in <3 seconds on 4G connection (Lighthouse performance test)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Create Account & Complete Onboarding (Priority: P2)

**Goal**: Implement Better-auth authentication with onboarding questionnaire for user profiling

**Independent Test**: Create account, complete questionnaire, confirm data persists across sessions, user can skip and revisit onboarding

### Database Schema

- [ ] T056 [US2] Create Alembic migration in backend/src/database/migrations/ for users table (id, email, password_hash, name, created_at, last_login)
- [ ] T057 [US2] Create Alembic migration for sessions table (Better-auth schema)
- [ ] T058 [US2] Create Alembic migration for onboarding_profiles table (user_id FK, software_background, hardware_background, learning_goals, ros_experience, ai_ml_experience, robotics_hardware_access, completed_at)
- [ ] T059 [US2] Run migrations to create tables in Neon Postgres

### Backend Authentication

- [ ] T060 [US2] Install and configure Better-auth library in backend/src/auth/config.py
- [ ] T061 [US2] Create backend/src/database/models.py User model with SQLAlchemy (id, email, password_hash, name, created_at, last_login)
- [ ] T062 [US2] Create backend/src/database/models.py OnboardingProfile model (user_id FK, software_background ENUM, hardware_background ENUM, learning_goals JSON, completed_at)
- [ ] T063 [US2] Implement POST /api/auth/signup endpoint in backend/src/auth/routes.py with bcrypt password hashing (10 rounds)
- [ ] T064 [US2] Implement POST /api/auth/signin endpoint in backend/src/auth/routes.py with session creation (httpOnly cookies, 7-day expiry)
- [ ] T065 [US2] Implement POST /api/auth/signout endpoint in backend/src/auth/routes.py
- [ ] T066 [US2] Implement GET /api/auth/session endpoint in backend/src/auth/routes.py to verify current session
- [ ] T067 [US2] Implement POST /api/auth/forgot-password endpoint in backend/src/auth/routes.py
- [ ] T068 [US2] Implement POST /api/auth/reset-password endpoint in backend/src/auth/routes.py
- [ ] T069 [US2] Add session validation middleware to backend/src/auth/middleware.py for protected routes
- [ ] T070 [US2] Configure CORS in backend/src/main.py to allow GitHub Pages origin with credentials

### User Profile Management

- [ ] T071 [US2] Create backend/src/users/models.py with Pydantic models (UserCreate, UserResponse, OnboardingProfileCreate, OnboardingProfileResponse)
- [ ] T072 [US2] Create backend/src/users/service.py with user CRUD operations (get, update, delete)
- [ ] T073 [US2] Implement GET /api/users/me endpoint in backend/src/users/routes.py to fetch current user profile
- [ ] T074 [US2] Implement PUT /api/users/me endpoint in backend/src/users/routes.py to update user profile
- [ ] T075 [US2] Implement DELETE /api/users/me endpoint in backend/src/users/routes.py for account deletion (GDPR compliance)

### Onboarding Endpoints

- [ ] T076 [US2] Implement POST /api/users/me/onboarding endpoint in backend/src/users/routes.py to save onboarding responses
- [ ] T077 [US2] Implement GET /api/users/me/onboarding endpoint in backend/src/users/routes.py to fetch onboarding status
- [ ] T078 [US2] Implement PUT /api/users/me/onboarding endpoint in backend/src/users/routes.py to update onboarding responses

### Frontend Authentication

- [ ] T079 [US2] Install Better-auth client SDK in frontend/package.json
- [ ] T080 [US2] Create frontend/src/hooks/useAuth.ts with Better-auth client wrapper (signup, signin, signout, getSession)
- [ ] T081 [US2] Create frontend/src/components/AuthButtons/index.tsx with Login and Signup buttons for Navbar
- [ ] T082 [US2] Create frontend/src/components/AuthButtons/LoginModal.tsx with email/password form
- [ ] T083 [US2] Create frontend/src/components/AuthButtons/SignupModal.tsx with email/password/name form
- [ ] T084 [US2] Add AuthButtons component to frontend/src/theme/Navbar/index.tsx
- [ ] T085 [US2] Implement session persistence and auto-refresh in frontend/src/hooks/useAuth.ts
- [ ] T086 [US2] Create protected route wrapper in frontend/src/components/ProtectedRoute.tsx

### Onboarding Flow

- [ ] T087 [US2] Create frontend/src/components/OnboardingModal/index.tsx with multi-step modal UI
- [ ] T088 [US2] Create frontend/src/components/OnboardingModal/Step1Software.tsx for software background questionnaire (Python, AI/ML, ROS experience dropdowns)
- [ ] T089 [US2] Create frontend/src/components/OnboardingModal/Step2Hardware.tsx for hardware background questionnaire (Linux, GPU, robotics hardware dropdowns)
- [ ] T090 [US2] Create frontend/src/components/OnboardingModal/Step3Goals.tsx for learning goals selection (checkboxes for goals)
- [ ] T091 [US2] Add onboarding modal trigger to frontend/ to show on first login (check onboarding completion status)
- [ ] T092 [US2] Implement "Skip for now" button in frontend/src/components/OnboardingModal/index.tsx
- [ ] T093 [US2] Create frontend/src/hooks/useOnboarding.ts to fetch and update onboarding data
- [ ] T094 [US2] Add user dashboard page at frontend/src/pages/dashboard.tsx to view/edit profile and onboarding

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Ask Chatbot Questions (Priority: P3)

**Goal**: Implement RAG pipeline with OpenAI embeddings, Qdrant vector search, and chatbot API

**Independent Test**: Ask technical questions via chatbot API, verify relevant answers with correct source citations, chat history persists

### Content Embedding Pipeline

- [ ] T095 [US3] Create backend/src/rag/embeddings.py with OpenAI text-embedding-3-small client wrapper
- [ ] T096 [US3] Create scripts/embed_content.py with content chunking logic (semantic splitting, 512 tokens per chunk)
- [ ] T097 [US3] Implement metadata extraction in scripts/embed_content.py (module, week, topic, difficulty, prerequisites, hardware_requirements, contains_code, code_language)
- [ ] T098 [US3] Implement Qdrant upsert logic in scripts/embed_content.py to store embeddings with metadata
- [ ] T099 [US3] Test embedding generation with sample chapters and verify quality
- [ ] T100 [US3] Run scripts/embed_content.py to embed all existing content (Weeks 1-5) into Qdrant
- [ ] T101 [US3] Update .github/workflows/embed-content.yml to trigger on docs/ changes and run scripts/embed_content.py

### RAG Retrieval System

- [ ] T102 [US3] Implement semantic search in backend/src/rag/retriever.py with Qdrant query (top_k=5, cosine similarity)
- [ ] T103 [US3] Add metadata filtering in backend/src/rag/retriever.py (module, difficulty, hardware requirements)
- [ ] T104 [US3] Implement context construction in backend/src/rag/retriever.py (format retrieved chunks with citations)
- [ ] T105 [US3] Create POST /api/chat/search endpoint in backend/src/chat/routes.py for testing retrieval (returns top chunks without LLM)

### Chat Database Schema

- [ ] T106 [US3] Create Alembic migration for conversations table (id, user_id FK, chapter_context, created_at, updated_at)
- [ ] T107 [US3] Create Alembic migration for messages table (id, conversation_id FK, role ENUM, content TEXT, selected_text_context TEXT, metadata JSON, created_at)
- [ ] T108 [US3] Create Alembic migration for feedback table (id, conversation_id FK, message_id FK, rating INT, feedback_text TEXT, created_at)
- [ ] T109 [US3] Run migrations to create chat tables in Neon Postgres

### Chat Backend Models

- [ ] T110 [P] [US3] Create backend/src/database/models.py Conversation model with SQLAlchemy
- [ ] T111 [P] [US3] Create backend/src/database/models.py Message model with SQLAlchemy
- [ ] T112 [P] [US3] Create backend/src/database/models.py Feedback model with SQLAlchemy
- [ ] T113 [US3] Create backend/src/chat/models.py with Pydantic models (ChatQueryRequest, ChatQueryResponse, MessageResponse, ConversationResponse)

### OpenAI Integration

- [ ] T114 [US3] Install OpenAI SDK in backend/requirements.txt
- [ ] T115 [US3] Create backend/src/rag/agents.py with OpenAI chat completion client wrapper
- [ ] T116 [US3] Implement RAG orchestration in backend/src/chat/service.py (retrieve from Qdrant, build context, call OpenAI, format response with citations)
- [ ] T117 [US3] Add streaming response support in backend/src/chat/service.py using OpenAI streaming API
- [ ] T118 [US3] Implement response caching in backend/src/chat/service.py (in-memory cache or Redis) to reduce duplicate API calls

### Chat Endpoints

- [ ] T119 [US3] Implement POST /api/chat/query endpoint in backend/src/chat/routes.py with RAG orchestration
- [ ] T120 [US3] Add conversation management in backend/src/chat/routes.py (create new conversation if conversation_id not provided)
- [ ] T121 [US3] Implement POST /api/chat/conversations endpoint to create new conversation thread
- [ ] T122 [US3] Implement GET /api/chat/conversations/:id/messages endpoint to fetch conversation history (last 20 messages with pagination)
- [ ] T123 [US3] Implement DELETE /api/chat/conversations/:id endpoint to delete conversation
- [ ] T124 [US3] Implement POST /api/chat/feedback endpoint in backend/src/chat/routes.py to record thumbs-up/down ratings

### Rate Limiting & Security

- [ ] T125 [US3] Implement rate limiting middleware in backend/src/chat/routes.py (10 queries per minute per authenticated user)
- [ ] T126 [US3] Add input validation in backend/src/chat/models.py with Pydantic (max query length 500 words)
- [ ] T127 [US3] Implement fallback responses in backend/src/chat/service.py when OpenAI API is unavailable
- [ ] T128 [US3] Add error handling and retry logic with exponential backoff in backend/src/rag/agents.py
- [ ] T129 [US3] Implement security event logging in backend/src/chat/routes.py (rate limit violations, failed queries)

**Checkpoint**: At this point, User Story 3 backend is complete and testable via API

---

## Phase 6: User Story 3 - Chatbot UI Widget (Priority: P3)

**Goal**: Build chatbot UI widget with floating button and slide-out panel in Docusaurus

**Independent Test**: Chatbot widget loads on all pages, user can send messages and see streaming responses, accessible via keyboard

### Chatbot Component Structure

- [ ] T130 [US3] Create frontend/src/components/ChatbotWidget/index.tsx with main widget container
- [ ] T131 [P] [US3] Create frontend/src/components/ChatbotWidget/FloatingButton.tsx with circular button UI
- [ ] T132 [P] [US3] Create frontend/src/components/ChatbotWidget/ChatPanel.tsx with slide-out panel container
- [ ] T133 [P] [US3] Create frontend/src/components/ChatbotWidget/ChatMessages.tsx with scrollable message list
- [ ] T134 [P] [US3] Create frontend/src/components/ChatbotWidget/ChatInput.tsx with auto-resize textarea
- [ ] T135 [P] [US3] Create frontend/src/components/ChatbotWidget/Message.tsx with user/assistant message bubbles

### Chat Functionality

- [ ] T136 [US3] Create frontend/src/hooks/useChat.ts to integrate with chat API endpoints
- [ ] T137 [US3] Implement message sending in frontend/src/hooks/useChat.ts with streaming response handling (Server-Sent Events)
- [ ] T138 [US3] Implement conversation history loading in frontend/src/hooks/useChat.ts (fetch last 20 messages on open)
- [ ] T139 [US3] Add typing indicators in frontend/src/components/ChatbotWidget/ChatMessages.tsx during response generation
- [ ] T140 [US3] Add "New Conversation" button in frontend/src/components/ChatbotWidget/ChatPanel.tsx
- [ ] T141 [US3] Implement chat history pagination in frontend/src/components/ChatbotWidget/ChatMessages.tsx (load older messages on scroll to top)

### UI/UX Polish

- [ ] T142 [US3] Style ChatbotWidget components with Docusaurus theme variables in frontend/src/components/ChatbotWidget/styles.module.css
- [ ] T143 [US3] Add open/close animations for ChatPanel slide-out in frontend/src/components/ChatbotWidget/ChatPanel.tsx
- [ ] T144 [US3] Implement error handling UI in frontend/src/components/ChatbotWidget/ChatPanel.tsx (retry button, error messages)
- [ ] T145 [US3] Add loading skeleton in frontend/src/components/ChatbotWidget/ChatMessages.tsx while history loads
- [ ] T146 [US3] Add source citations display in frontend/src/components/ChatbotWidget/Message.tsx (module + week links)
- [ ] T147 [US3] Implement thumbs-up/down feedback buttons in frontend/src/components/ChatbotWidget/Message.tsx

### Integration & Accessibility

- [ ] T148 [US3] Add ChatbotWidget to Docusaurus Root wrapper in frontend/src/theme/Root.tsx to display on all pages
- [ ] T149 [US3] Integrate with useAuth hook in frontend/src/components/ChatbotWidget/index.tsx (show login prompt if not authenticated)
- [ ] T150 [US3] Add keyboard navigation support in frontend/src/components/ChatbotWidget/ChatPanel.tsx (Esc to close, Tab navigation)
- [ ] T151 [US3] Add ARIA labels and roles in frontend/src/components/ChatbotWidget/ components for screen readers
- [ ] T152 [US3] Test chatbot widget on mobile devices and fix responsive issues (viewport units, touch targets)

**Checkpoint**: At this point, User Story 3 should be fully functional (backend + frontend)

---

## Phase 7: User Story 4 - Select Text & Ask Contextual Questions (Priority: P4)

**Goal**: Implement text selection handler to automatically open chatbot with selected text as context

**Independent Test**: Select text on content page, chatbot opens automatically with context badge, queries reference selected text

### Text Selection Handler

- [ ] T153 [US4] Create frontend/src/components/TextSelectionHandler/index.tsx with mouseup event listener
- [ ] T154 [US4] Implement selection length validation in frontend/src/components/TextSelectionHandler/index.tsx (minimum 10 characters)
- [ ] T155 [US4] Create frontend/src/hooks/useTextSelection.ts to manage selected text state
- [ ] T156 [US4] Dispatch custom event with selected text in frontend/src/components/TextSelectionHandler/index.tsx
- [ ] T157 [US4] Add TextSelectionHandler to Docusaurus Root wrapper in frontend/src/theme/Root.tsx

### Chatbot Context Integration

- [ ] T158 [US4] Update frontend/src/components/ChatbotWidget/index.tsx to listen for text selection events
- [ ] T159 [US4] Create frontend/src/components/ChatbotWidget/ContextBadge.tsx to display selected text in chatbot panel
- [ ] T160 [US4] Implement auto-open chatbot on text selection in frontend/src/components/ChatbotWidget/index.tsx
- [ ] T161 [US4] Add "Clear context" button (X) in frontend/src/components/ChatbotWidget/ContextBadge.tsx
- [ ] T162 [US4] Update frontend/src/hooks/useChat.ts to include selected_text in ChatQueryRequest
- [ ] T163 [US4] Preserve syntax highlighting for code block selections in frontend/src/components/ChatbotWidget/ContextBadge.tsx

### Backend Context Handling

- [ ] T164 [US4] Update backend/src/chat/models.py ChatQueryRequest to include selected_text field
- [ ] T165 [US4] Modify backend/src/chat/service.py to prioritize selected text in retrieval (boost relevance of chunks containing selected text)
- [ ] T166 [US4] Update backend/src/rag/retriever.py to include selected_text in query context for OpenAI

**Checkpoint**: At this point, User Story 4 should be fully functional

---

## Phase 8: User Story 5 - Personalize Chapter Content (Priority: P5)

**Goal**: Implement personalization button to trigger adaptive chatbot explanations based on user profile

**Independent Test**: Users with different backgrounds receive appropriately tailored explanations when clicking "Personalize This Chapter"

### Personalize Button Component

- [ ] T167 [US5] Create frontend/src/components/PersonalizeButton/index.tsx with button UI
- [ ] T168 [US5] Add PersonalizeButton to Docusaurus DocItem layout in frontend/src/theme/DocItem/Layout/index.tsx (swizzle if needed)
- [ ] T169 [US5] Position button at chapter start in frontend/src/components/PersonalizeButton/index.tsx
- [ ] T170 [US5] Add onClick handler to trigger chatbot with personalization request in frontend/src/components/PersonalizeButton/index.tsx

### Backend Personalization Service

- [ ] T171 [US5] Create backend/src/personalization/models.py with Pydantic models (PersonalizationRequest, PersonalizationResponse)
- [ ] T172 [US5] Create backend/src/personalization/service.py with profile-based prompt builder
- [ ] T173 [US5] Implement GET /api/personalization/recommendations endpoint in backend/src/personalization/routes.py
- [ ] T174 [US5] Build adaptive prompt templates in backend/src/personalization/service.py for different backgrounds (beginner, intermediate, advanced)
- [ ] T175 [US5] Add hardware-specific recommendations in backend/src/personalization/service.py (RTX GPU vs Jetson vs cloud alternatives)

### RAG Integration

- [ ] T176 [US5] Update backend/src/chat/service.py to accept user profile context in queries
- [ ] T177 [US5] Add difficulty level filtering in backend/src/rag/retriever.py based on user software background
- [ ] T178 [US5] Implement proactive explanation generation in backend/src/personalization/service.py (triggered by PersonalizeButton)
- [ ] T179 [US5] Update frontend/src/components/ChatbotWidget/index.tsx to handle personalization requests and open with proactive message

### Profile Management

- [ ] T180 [US5] Add "Edit Preferences" link in frontend/src/pages/dashboard.tsx to update onboarding responses
- [ ] T181 [US5] Update frontend/src/components/OnboardingModal/index.tsx to be reusable for profile editing
- [ ] T182 [US5] Implement profile change detection in backend/src/personalization/service.py to adjust future personalization

**Checkpoint**: At this point, User Story 5 should be fully functional

---

## Phase 9: User Story 6 - View Chat History (Priority: P6)

**Goal**: Persist and load chat history for authenticated users

**Independent Test**: Ask questions, close browser, reopen, verify previous messages are loaded

### Chat History Implementation

- [ ] T183 [US6] Implement chat history loading in frontend/src/hooks/useChat.ts on chatbot open (GET /api/chat/conversations/:id/messages)
- [ ] T184 [US6] Add conversation list UI in frontend/src/components/ChatbotWidget/ConversationList.tsx
- [ ] T185 [US6] Implement GET /api/chat/conversations endpoint in backend/src/chat/routes.py to list user's conversations
- [ ] T186 [US6] Add conversation switching in frontend/src/components/ChatbotWidget/ChatPanel.tsx
- [ ] T187 [US6] Implement "New Conversation" functionality with conversation_id creation
- [ ] T188 [US6] Add pagination for long conversations in frontend/src/components/ChatbotWidget/ChatMessages.tsx (load older messages on scroll up)
- [ ] T189 [US6] Implement DELETE conversation functionality in frontend/src/components/ChatbotWidget/ConversationList.tsx
- [ ] T190 [US6] Add "Delete All History" option in frontend/src/pages/dashboard.tsx

### Context Preservation

- [ ] T191 [US6] Store conversation context in backend/src/database/models.py Message metadata (chapter, selected text, personalization context)
- [ ] T192 [US6] Implement context restoration in backend/src/chat/service.py when loading conversation history
- [ ] T193 [US6] Add conversation summary generation in backend/src/chat/service.py for long conversations (display in ConversationList)

**Checkpoint**: At this point, User Story 6 should be fully functional

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Content Expansion

- [ ] T194 [P] Develop Week 6 content in frontend/docs/02-simulation/week-06.md (Gazebo basics)
- [ ] T195 [P] Develop Week 7 content in frontend/docs/02-simulation/week-07.md (Unity integration)
- [ ] T196 [P] Develop Week 8 content in frontend/docs/03-isaac/week-08.md (NVIDIA Isaac basics)
- [ ] T197 [P] Develop Week 9 content in frontend/docs/03-isaac/week-09.md (Isaac Sim workflows)
- [ ] T198 [P] Develop Week 10 content in frontend/docs/03-isaac/week-10.md (Isaac advanced topics)
- [ ] T199 [P] Develop Week 11 content in frontend/docs/04-vla/week-11.md (VLA introduction)
- [ ] T200 [P] Develop Week 12 content in frontend/docs/04-vla/week-12.md (VLA implementation)
- [ ] T201 [P] Develop Week 13 content in frontend/docs/04-vla/week-13.md (VLA advanced topics)
- [ ] T202 [P] Add assessment content to frontend/docs/assessments/ with rubrics and starter code
- [ ] T203 Run scripts/embed_content.py to embed Weeks 6-13 content into Qdrant

### Performance Optimization

- [ ] T204 [P] Optimize frontend bundle size in frontend/docusaurus.config.js (code splitting, lazy loading)
- [ ] T205 [P] Add image optimization in frontend/static/img/ (compress, WebP format, lazy loading)
- [ ] T206 Implement response caching with Redis in backend/src/chat/service.py (60%+ cache hit rate target)
- [ ] T207 Optimize Qdrant queries in backend/src/rag/retriever.py (tune top_k, similarity threshold, HNSW parameters)
- [ ] T208 Add request debouncing in frontend/src/hooks/useChat.ts to prevent duplicate queries
- [ ] T209 Run Lighthouse performance audit and achieve LCP <2.5s, FCP <1.5s

### Monitoring & Observability

- [ ] T210 [P] Setup Sentry error tracking in backend/src/main.py and frontend/src/
- [ ] T211 [P] Setup PostHog analytics in frontend/src/ (opt-in only, GDPR compliant)
- [ ] T212 Add structured logging in backend/src/core/logging.py (JSON format, request IDs)
- [ ] T213 Create monitoring dashboard for API metrics (response times, error rates, cache hit rates)
- [ ] T214 Add cost monitoring for OpenAI API usage in backend/src/rag/agents.py (alert if approaching budget)

### Security Hardening

- [ ] T215 Run dependency security scan with npm audit and pip-audit, fix vulnerabilities
- [ ] T216 [P] Add input sanitization in backend/src/chat/models.py to prevent injection attacks
- [ ] T217 [P] Implement Content Security Policy (CSP) headers in frontend/docusaurus.config.js
- [ ] T218 Add rate limiting for authentication endpoints in backend/src/auth/routes.py (prevent brute force)
- [ ] T219 Review CORS configuration in backend/src/main.py (strict origin whitelist)
- [ ] T220 Implement API request signing in frontend/src/hooks/useChat.ts to prevent CSRF

### Documentation

- [ ] T221 [P] Create frontend/README.md with Docusaurus setup and development instructions
- [ ] T222 [P] Create backend/README.md with FastAPI setup and API documentation
- [ ] T223 [P] Create scripts/README.md with embedding pipeline documentation
- [ ] T224 Update root README.md with architecture overview and deployment guide
- [ ] T225 Create specs/001-rag-textbook-platform/quickstart.md with local development guide
- [ ] T226 [P] Document API endpoints in specs/001-rag-textbook-platform/contracts/ (OpenAPI spec)
- [ ] T227 Create CONTRIBUTING.md with code contribution guidelines

### Accessibility & Testing

- [ ] T228 Run WCAG 2.1 AA compliance audit and fix violations
- [ ] T229 Test with keyboard navigation only (no mouse) and fix issues
- [ ] T230 Test with screen readers (NVDA, JAWS, VoiceOver) and fix accessibility issues
- [ ] T231 Perform cross-browser testing (Chrome, Firefox, Safari latest 2 versions)
- [ ] T232 Perform mobile testing on iOS and Android devices
- [ ] T233 Run load testing with 100 concurrent users and fix performance bottlenecks

### Deployment Validation

- [ ] T234 Validate GitHub Actions workflows execute successfully for all branches
- [ ] T235 Verify Render backend deployment with all environment variables configured
- [ ] T236 Verify GitHub Pages frontend deployment with correct base URL
- [ ] T237 Test production environment end-to-end (signup, onboarding, chat, personalization, history)
- [ ] T238 Setup uptime monitoring for production site (99.5% uptime target)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-9)**: All depend on Foundational phase completion
  - US1 (P1): Can start after Foundational (Phase 2) - No dependencies on other stories
  - US2 (P2): Can start after Foundational (Phase 2) - No dependencies on other stories
  - US3 (P3): Can start after Foundational (Phase 2) - Depends on US1 (content must exist for embeddings)
  - US4 (P4): Depends on US3 completion (chatbot must be functional)
  - US5 (P5): Depends on US2 + US3 completion (onboarding + chatbot must be functional)
  - US6 (P6): Depends on US3 completion (chatbot must be functional)
- **Polish (Phase 10)**: Depends on all desired user stories being complete

### Critical Path

1. **Weeks 1-2**: Phase 1 (Setup) + Phase 2 (Foundational)
2. **Weeks 2-4**: Phase 3 (US1 - Content) can run parallel with Phase 4 (US2 - Auth)
3. **Weeks 4-5**: Phase 5 (US3 Part 1 - Embedding) - depends on US1 content
4. **Week 5**: Phase 5 (US3 Part 2 - Chat Backend) - depends on embedding pipeline
5. **Week 6**: Phase 6 (US3 Part 3 - Chat UI) - depends on chat backend
6. **Weeks 6-7**: Phase 7 (US4 - Text Selection) - depends on US3 completion
7. **Weeks 7-8**: Phase 8 (US5 - Personalization) - depends on US2 + US3 completion
8. **Week 8**: Phase 9 (US6 - Chat History) - depends on US3 completion
9. **Weeks 8-9**: Phase 10 (Polish)
10. **Week 10+**: Production launch

### Parallel Opportunities

- **After Setup (Phase 1)**: All Foundational tasks marked [P] can run in parallel
- **After Foundational (Phase 2)**: US1 (Content) and US2 (Auth) can run in parallel
- **Within each phase**: All tasks marked [P] can run in parallel
- **After US3 complete**: US4, US6 can start in parallel
- **After US2 + US3 complete**: US5 can start

### MVP Milestones

- **MVP v0.1** (Week 4): Phase 1 + Phase 2 + Phase 3 (US1 only) - Static textbook deployed
- **MVP v0.2** (Week 5): + Phase 4 (US2) - Authentication and onboarding functional
- **MVP v0.3** (Week 6): + Phase 5 + Phase 6 (US3) - RAG chatbot functional
- **MVP v1.0** (Week 10): All 6 user stories + Polish complete - Full production launch

---

## Implementation Strategy

### MVP First (US1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Browse Content)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy to GitHub Pages as MVP v0.1

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add US1 → Test independently → Deploy (MVP v0.1)
3. Add US2 → Test independently → Deploy (MVP v0.2)
4. Add US3 → Test independently → Deploy (MVP v0.3)
5. Add US4, US5, US6 → Test independently → Deploy (MVP v1.0)
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: US1 (Content development)
   - Developer B: US2 (Authentication)
3. After US1 + US2:
   - Developer A: US3 Part 1 (Embedding pipeline)
   - Developer B: US3 Part 2 (Chat backend)
4. After US3:
   - Developer A: US4 (Text selection) + US6 (History)
   - Developer B: US5 (Personalization)
5. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability (US1-US6)
- Setup tasks have NO [Story] label (shared infrastructure)
- Foundational tasks have NO [Story] label (blocking prerequisites)
- Polish tasks have NO [Story] label (cross-cutting concerns)
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
