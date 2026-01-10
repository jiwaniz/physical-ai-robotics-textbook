---
id: 0001
title: Create project constitution
stage: constitution
date: 2026-01-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Zahra
command: /sp.constitution
labels: ["constitution", "project-setup", "governance", "principles"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - None (constitution is documentation)
---

## Prompt

/sp.constitution based on above conversation create constitution

Context from conversation:
- Building a Docusaurus-based textbook for "Physical AI & Humanoid Robotics"
- 13-week course with 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- Integrated RAG chatbot using OpenAI Agents SDK, FastAPI, Neon Postgres, Qdrant
- Better-auth for authentication with onboarding flow (software/hardware background)
- Personalization via chatbot (adapt responses based on user profile)
- Text selection feature for contextual help
- Deploy to GitHub Pages (frontend) + Render (backend)
- Target audience: Students with AI/ML background learning robotics

## Response snapshot

Created comprehensive constitution (v1.0.0) with 6 core principles:

1. **Educational Excellence** - Content must be accurate, comprehensive, pedagogically sound
   - Technical accuracy for ROS 2, NVIDIA Isaac, Gazebo, VLA
   - Tested code examples, progressive explanations, troubleshooting guides

2. **Universal Accessibility (Personalization)** - Accessible to all skill levels
   - Onboarding captures software (Python, AI/ML, ROS) and hardware (Linux, GPU, robotics) background
   - RAG chatbot tailors responses based on user profile
   - WCAG 2.1 AA accessibility standards
   - Alternative paths for different hardware setups (RTX GPU vs Jetson vs cloud)

3. **RAG-Powered Support** - Accurate, context-aware chatbot assistance
   - Semantic search retrieves relevant content chunks
   - Cite sources (module, week, section)
   - Handle technical queries (ROS 2 commands, URDF, Isaac Sim errors)
   - Rate limiting (10 queries/min) and fallback mechanisms

4. **Hands-On Learning (Reproducibility)** - All code examples must be reproducible
   - Complete code context (imports, dependencies, environment)
   - Version pinning (ROS 2 Humble, Isaac 2023.1.1, Ubuntu 22.04)
   - "Works on my machine" is NOT acceptable - test on clean install
   - Clear rubrics and starter code for assessments

5. **Performance First** - Fast loading, efficient costs
   - Frontend: FCP <1.5s, LCP <2.5s, bundle <200KB gzipped
   - Backend: p95 latency <500ms, chat first token <1s
   - Cost: OpenAI API <$100/month for 100 users, use batch API, cache responses

6. **Security & Privacy** - Data protection and secure auth
   - Better-Auth with httpOnly cookies (NOT localStorage)
   - HTTPS enforced, CORS whitelist, rate limiting
   - GDPR compliance, no third-party tracking

Additional sections:
- **Technical Standards**: Technology stack (Docusaurus, FastAPI, Better-Auth, Neon, Qdrant, OpenAI), code quality standards
- **Content Development Standards**: Peer review, content organization (4 modules × 13 weeks), code example requirements
- **User Experience Requirements**: Navigation, mobile responsiveness, accessibility
- **Governance**: Amendment process, compliance enforcement, versioning rules

## Outcome

- ✅ Impact: Initial project constitution established, governance framework in place
- 🧪 Tests: N/A (documentation artifact)
- 📁 Files: Created .specify/memory/constitution.md (v1.0.0)
- 🔁 Next prompts: Review templates for alignment, create initial ADRs for architecture decisions, exit plan mode and begin implementation
- 🧠 Reflection: Constitution is comprehensive and tailored to educational robotics platform. Principles directly address key challenges: technical accuracy, diverse student backgrounds, hardware complexity, cost management, and security.

## Evaluation notes (flywheel)

- Failure modes observed: None (initial creation)
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Monitor adherence during implementation; may need to adjust performance budgets or add specific ROS 2/robotics principles based on real development challenges
