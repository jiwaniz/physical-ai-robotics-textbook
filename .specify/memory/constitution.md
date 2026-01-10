<!--
Sync Impact Report:
Version: 0.0.0 → 1.0.0
Change Type: Initial constitution creation
Modified Principles: N/A (initial creation)
Added Sections:
  - Core Principles (6 principles)
  - Technical Standards
  - Content Development Standards
  - User Experience Requirements
  - Governance
Removed Sections: N/A
Templates Requiring Updates:
  ✅ constitution.md (this file)
  ⚠ .specify/templates/plan-template.md (review constitution checks)
  ⚠ .specify/templates/spec-template.md (align with educational content standards)
  ⚠ .specify/templates/tasks-template.md (ensure testing/quality tasks reflect principles)
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Platform Constitution

## Core Principles

### I. Educational Excellence

**Content MUST be accurate, comprehensive, and pedagogically sound.**

- All technical content (ROS 2, NVIDIA Isaac, Gazebo, VLA) MUST be technically accurate and up-to-date
- Each module MUST include: learning objectives, conceptual explanations, step-by-step tutorials, and assessments
- Code examples MUST be tested and working before publication
- Complex concepts MUST be explained progressively (fundamentals → advanced)
- Hardware requirements MUST be clearly stated for each module
- Troubleshooting guides MUST be provided for common issues

**Rationale:** Students trust educational platforms to teach correct information. Inaccurate or outdated content in robotics can lead to wasted time, broken systems, and lost learning opportunities. This is especially critical for hardware-intensive topics where mistakes are costly.

### II. Universal Accessibility (Personalization)

**Content MUST be accessible and personalized for all skill levels (beginner to advanced).**

- Onboarding MUST capture user background: software skills (Python, AI/ML, ROS), hardware skills (Linux, GPU, robotics)
- RAG chatbot MUST tailor responses based on user profile (beginners get simplified explanations, advanced users get technical depth)
- "Personalize" buttons at chapter starts MUST trigger context-appropriate explanations
- Content MUST NOT assume prerequisites without explicitly stating them
- Alternative paths MUST be provided for students with different hardware (RTX GPU vs Jetson vs cloud)
- WCAG 2.1 AA accessibility standards MUST be met (keyboard navigation, screen readers, contrast ratios)

**Rationale:** The course bridges AI and robotics - students come from diverse backgrounds. AI experts may not know Linux/ROS, while roboticists may not know transformers/LLMs. Personalization prevents frustration and dropout.

### III. RAG-Powered Support (Intelligent Assistance)

**The RAG chatbot MUST provide accurate, context-aware, and helpful assistance.**

- Semantic search MUST retrieve relevant content chunks from the textbook
- Responses MUST cite sources (module, week, section) for verification
- User-selected text MUST be prioritized in context for targeted help
- Chatbot MUST handle technical queries: ROS 2 commands, URDF syntax, Isaac Sim errors, troubleshooting
- Response quality MUST be monitored via user feedback (thumbs up/down)
- Fallback mechanisms MUST exist when OpenAI API fails (cached responses, error messages)
- Rate limiting MUST prevent abuse (10 queries/min per user)

**Rationale:** This is a highly technical course with many potential failure points (physics simulation crashes, ROS 2 errors, hardware incompatibilities). Students need instant, accurate help. A poor chatbot is worse than no chatbot.

### IV. Hands-On Learning (Reproducibility)

**All code examples, tutorials, and projects MUST be reproducible by students.**

- Code examples MUST include complete context (imports, dependencies, environment setup)
- ROS 2 packages MUST follow standard structure (src/, launch/, package.xml, setup.py)
- URDF files MUST be valid and tested in Gazebo
- Docker containers or VM images SHOULD be provided for complex setups
- Version pinning MUST be used (ROS 2 Humble, NVIDIA Isaac 2023.1.1, etc.)
- "Works on my machine" is NOT acceptable - test on clean Ubuntu 22.04 install
- Assessments (ROS 2 project, Gazebo simulation, Isaac pipeline, Capstone) MUST have clear rubrics and starter code

**Rationale:** Robotics development is notoriously fragile - small version mismatches break everything. Students waste hours debugging environment issues. Reproducibility is non-negotiable for educational content.

### V. Performance First (Speed & Cost Optimization)

**The platform MUST load fast, respond quickly, and minimize operational costs.**

- **Frontend Performance:**
  - First Contentful Paint (FCP) < 1.5s
  - Largest Contentful Paint (LCP) < 2.5s
  - JavaScript bundle < 200KB gzipped
  - Images MUST be optimized (WebP format, lazy loading)

- **Backend Performance:**
  - API p95 latency < 500ms
  - Chat first token < 1s (streaming responses)
  - Qdrant search < 100ms
  - Database queries < 50ms

- **Cost Management:**
  - Monthly OpenAI API costs MUST NOT exceed $100 (for 100 active users)
  - Embedding generation MUST use batch API (50% discount)
  - Chat responses MUST be cached (avoid duplicate OpenAI calls)
  - Monitor token usage and alert on anomalies

**Rationale:** Students access this platform on various devices (laptops, tablets, weak connections). Slow loading kills engagement. OpenAI API costs can spiral - we're optimizing for sustainability and scalability within free/low-cost tiers.

### VI. Security & Privacy (Data Protection)

**User data MUST be protected; authentication MUST be secure.**

- **Authentication:**
  - Better-Auth with httpOnly session cookies (NOT localStorage tokens)
  - bcrypt password hashing (minimum 10 rounds)
  - HTTPS enforced on all environments
  - Session expiry: 7 days with sliding renewal

- **API Security:**
  - CORS with strict origin whitelist
  - Rate limiting on all endpoints (protect against abuse)
  - Input validation with Pydantic (prevent injection attacks)
  - SQL injection prevention via SQLAlchemy parameterized queries
  - XSS prevention via React auto-escaping + CSP headers

- **Privacy:**
  - GDPR compliance: users can export/delete their data
  - Chat history stored only for logged-in users (optional)
  - Onboarding responses used only for personalization (explicit consent)
  - No third-party tracking (analytics opt-in)
  - API keys NEVER committed to Git (use environment variables)

**Rationale:** Educational platforms handle student data - we have a duty to protect it. Security breaches destroy trust. Even though this is a free educational resource, privacy is non-negotiable.

---

## Technical Standards

### Technology Stack (Non-Negotiable)

- **Frontend:** Docusaurus 3.x (static site generator) - MUST NOT use Next.js, Gatsby, or custom builds
- **Backend:** FastAPI 0.115+ (async Python) - MUST NOT use Flask, Django, or Node.js
- **Auth:** Better-Auth v1.x (self-hosted) - MUST NOT use Auth0, Clerk, or Firebase Auth
- **Databases:**
  - Neon Serverless Postgres (user data) - MUST NOT use MongoDB, MySQL, or Firebase
  - Qdrant Cloud (vectors) - MUST NOT use Pinecone, Weaviate, or pgvector
- **AI:** OpenAI Agents SDK / ChatKit (RAG) - MUST NOT use LangChain or LlamaIndex for MVP
- **Deployment:**
  - Frontend: GitHub Pages (free tier)
  - Backend: Render (free tier)
  - CI/CD: GitHub Actions

**Rationale:** These choices were made in ADRs (001-005). Deviations require new ADRs with trade-off analysis.

### Code Quality

- **Python (Backend):**
  - Type hints mandatory (PEP 484)
  - Black formatter (line length 100)
  - isort for imports
  - Pylint score > 8.0
  - Pytest for all tests

- **TypeScript (Frontend):**
  - Strict mode enabled
  - ESLint + Prettier
  - React hooks (functional components only)
  - No PropTypes (use TypeScript interfaces)

- **Testing:**
  - Backend: 80%+ unit test coverage
  - E2E tests for critical flows (signup, chat, personalization)
  - Integration tests for RAG pipeline
  - No production deployment without passing tests

---

## Content Development Standards

### Content Accuracy & Review

- All technical content MUST be peer-reviewed by someone with robotics/ROS 2 experience
- Code examples MUST be tested in the target environment (Ubuntu 22.04 + ROS 2 Humble)
- Hardware requirements MUST be verified (don't claim "works on Jetson Nano" without testing)
- External references (NVIDIA docs, ROS 2 docs) MUST include version and last-checked date
- Outdated content MUST be flagged and updated within 1 month of detection

### Content Organization

- **Structure:**
  - Weeks 1-2: Introduction to Physical AI
  - Module 1 (Weeks 3-5): ROS 2 Fundamentals
  - Module 2 (Weeks 6-7): Gazebo & Unity Simulation
  - Module 3 (Weeks 8-10): NVIDIA Isaac Platform
  - Module 4 (Weeks 11-13): VLA & Capstone

- **Each Week MUST Include:**
  - Learning outcomes (3-5 bullet points)
  - Conceptual overview (why this matters)
  - Step-by-step tutorial
  - Code examples (commented, runnable)
  - Common errors & troubleshooting
  - Assessment criteria (if applicable)

- **Each Code Example MUST Include:**
  - Prerequisites (e.g., "Requires ROS 2 Humble installed")
  - Hardware requirements (e.g., "RTX GPU recommended")
  - Complete code (no "..." placeholders)
  - Expected output
  - How to verify it worked

### Content Updates

- Major framework updates (ROS 2 new distro, Isaac Sim 2024.x) trigger content review
- Student-reported errors MUST be triaged within 48 hours
- Content roadmap maintained in `docs/content-roadmap.md`

---

## User Experience Requirements

### Navigation & Discoverability

- Sidebar navigation MUST reflect module/week structure
- Search MUST index all content (Algolia DocSearch or local)
- Breadcrumbs MUST show current location
- "Next/Previous" buttons at bottom of each page
- Progress tracking (optional for v2)

### Mobile Responsiveness

- All pages MUST be usable on mobile (320px width minimum)
- Chatbot widget MUST adapt to small screens (slide-up panel on mobile)
- Code blocks MUST be horizontally scrollable (not cut off)
- Touch targets MUST be 44×44px minimum

### Accessibility

- Semantic HTML (proper heading hierarchy h1→h2→h3)
- Alt text for all images
- Keyboard navigation for all interactive elements
- Focus indicators visible
- Color contrast WCAG AA compliant
- Screen reader tested with NVDA/JAWS

---

## Governance

### Amendment Process

This constitution is a living document. Amendments follow this process:

1. **Proposal:** Create ADR in `history/adr/` describing proposed change
2. **Discussion:** Review trade-offs, alternatives, and impact
3. **Approval:** Requires sign-off from project maintainer
4. **Migration:** Update affected templates, docs, and code
5. **Version Bump:**
   - MAJOR (X.0.0): Principle removal or incompatible change
   - MINOR (X.Y.0): New principle or section added
   - PATCH (X.Y.Z): Clarifications, wording fixes

### Compliance & Enforcement

- All PRs MUST include a constitution compliance check
- Plan template includes "Constitution Gates" section
- Violations MUST be documented and fixed within sprint
- Repeated violations trigger architecture review

### Runtime Guidance

- For day-to-day development guidance, see `CLAUDE.md`
- For architecture decisions, see `history/adr/`
- For feature planning, see `.specify/templates/plan-template.md`

---

**Version**: 1.0.0 | **Ratified**: 2026-01-05 | **Last Amended**: 2026-01-05
