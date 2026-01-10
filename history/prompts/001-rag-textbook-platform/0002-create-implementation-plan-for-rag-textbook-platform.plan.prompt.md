---
id: 0002
title: Create implementation plan for RAG textbook platform
stage: plan
date: 2026-01-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-textbook-platform
branch: 001-rag-textbook-platform
user: Zahra
command: /sp.plan
labels: ["planning", "architecture", "rag-chatbot", "docusaurus", "fastapi", "better-auth"]
links:
  spec: specs/001-rag-textbook-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-rag-textbook-platform/plan.md
tests:
 - None (planning artifact)
---

## Prompt

/sp.plan

Context:
- Constitution v1.0.0 with 6 principles (Educational Excellence, Universal Accessibility, RAG-Powered Support, Hands-On Learning, Performance First, Security & Privacy)
- Feature spec completed with 58 functional requirements, 24 success criteria, 6 user stories
- Physical AI & Humanoid Robotics textbook platform
- 13-week course across 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- Technology stack: Docusaurus, FastAPI, Better-Auth, Neon Postgres, Qdrant, OpenAI Agents SDK
- Features: RAG chatbot, personalization via onboarding, text selection, GitHub Pages + Render deployment

## Response snapshot

Created comprehensive implementation plan with:

**Technical Context:**
- Languages: Python 3.11+, Node.js 18+, React 18+, TypeScript
- Dependencies: FastAPI 0.115+, Docusaurus 3.x, Better-Auth v1.x, OpenAI SDK, SQLAlchemy, Qdrant client
- Storage: Neon Serverless Postgres (users, chat, onboarding), Qdrant Cloud (1536-dim embeddings)
- Performance Goals: Frontend FCP <1.5s, API p95 <500ms, chat first token <1s
- Constraints: <$100/month for 100 users, free tier optimization

**Constitution Check:**
- ✅ All 6 principles satisfied
- Educational Excellence: FR-002 (learning outcomes, code examples), tested code assumption
- Universal Accessibility: FR-007-013 (Better-Auth), FR-014-018 (onboarding), WCAG 2.1 AA
- RAG-Powered Support: FR-019-028 (semantic search, citations, rate limiting)
- Hands-On Learning: FR-001-006 (reproducible examples), version pinning
- Performance First: FR-047-051 (performance budgets), SC-010-017 (cost metrics)
- Security & Privacy: FR-052-058 (HTTPS, CORS, httpOnly cookies, GDPR)

**Project Structure:**
- frontend/ (Docusaurus: docs/, src/components/, src/hooks/, docusaurus.config.js)
- backend/ (FastAPI: src/auth/, src/chat/, src/rag/, src/database/, tests/)
- scripts/ (embedding pipeline, database setup)
- Content organized: 00-introduction/ (Weeks 1-2), 01-ros2/ (Weeks 3-5), 02-simulation/ (Weeks 6-7), 03-isaac/ (Weeks 8-10), 04-vla/ (Weeks 11-13)

**Implementation Phases (10 phases, 10-12 weeks):**
- Phase 0: Project Setup & Infrastructure (Week 1) - repo structure, CI/CD, databases
- Phase 1: Authentication & User Management (Week 2) - Better-Auth integration
- Phase 2: Content Development & Basic Site (Weeks 2-4) - expand outline, Docusaurus setup
- Phase 3: Onboarding Flow (Week 3) - multi-step modal, profile storage
- Phase 4: RAG System - Content Embedding (Week 4) - chunking, OpenAI embeddings, Qdrant
- Phase 5: RAG System - Chat Integration (Week 5) - Agents SDK, retrieval, history
- Phase 6: Chatbot UI Widget (Week 6) - floating button, slide-out panel
- Phase 7: Text Selection Feature (Weeks 6-7) - selection handler, context-aware queries
- Phase 8: Personalization System (Weeks 7-8) - personalize button, adaptive responses
- Phase 9: Optimization & Polish (Weeks 8-9) - caching, analytics, feedback
- Phase 10: Launch & Iteration (Week 10+) - production deployment, beta testing

**Key Architectural Decisions:**
1. Docusaurus vs Next.js → Docusaurus (documentation-focused, GitHub Pages support)
2. Better-Auth hybrid strategy → Server in FastAPI + client SDK (secure httpOnly cookies)
3. Pre-build embedding pipeline → CI/CD triggered (predictable costs, no cold starts)
4. Chatbot UI/UX → Floating button + slide-out panel (Intercom-style)
5. Personalization strategy → Dynamic prompts (no content duplication)
6. Multi-environment deployment → dev/staging/production branches

**Risk Mitigation:**
- OpenAI rate limits → Request queue, caching, graceful degradation
- Poor embedding quality → A/B test chunking, tune retrieval
- Unhelpful chatbot → Feedback collection, prompt iteration
- Cost overruns → Hard spending limits, daily monitoring, aggressive caching

## Outcome

- ✅ Impact: Comprehensive implementation plan ready; all architectural decisions documented and constitution-compliant
- 🧪 Tests: N/A (planning artifact)
- 📁 Files: Created plan.md (530 lines) with technical context, constitution check, project structure, 10 phases
- 🔁 Next prompts: `/sp.tasks` to generate actionable task breakdown, or create supporting artifacts (research.md, data-model.md, contracts/, quickstart.md)
- 🧠 Reflection: Plan bridges specification to implementation with clear phases, acceptance criteria, and risk mitigation. All 6 constitution principles mapped and verified. Architecture decisions favor simplicity (Docusaurus over Next.js), cost efficiency (pre-build embedding), and security (Better-Auth httpOnly cookies). Ready for task generation.

## Evaluation notes (flywheel)

- Failure modes observed: None - all architectural decisions aligned with constitution and functional requirements
- Graders run and results (PASS/FAIL): Constitution compliance check: PASS (all 6 principles satisfied)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): During task generation (/sp.tasks), validate that Phase 0 tasks can be executed independently to unblock parallel development streams (frontend setup vs backend setup vs database provisioning)
