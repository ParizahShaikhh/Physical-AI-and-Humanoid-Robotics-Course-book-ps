---
id: 1
title: "Update Project Constitution"
stage: constitution
date_iso: "2025-12-16"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "master"
user: "user"
command: "/sp.constitution"
labels: ["constitution", "spec-driven-development", "ai-book", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests:
  - "constitution update verified"
---

## Prompt

/sp.constitution

Project: AI-Spec–Driven Interactive Book with Embedded RAG Chatbot

Purpose:
Design, author, and deploy a production-grade, AI-authored technical book using Spec-Kit Plus and Claude Code, published via Docusaurus and GitHub Pages, with an integrated Retrieval-Augmented Generation (RAG) chatbot capable of contextual, selective, and personalized responses based on book content.

Core Principles:
- Spec-driven development: all features, content, and integrations must be defined, implemented, and validated through executable specifications.
- AI-native authorship: Claude Code is the primary authoring and refactoring agent; human input is supervisory and architectural.
- Technical accuracy: all technical explanations, code, and architectural claims must be verifiable and internally consistent.
- User-centric intelligence: content, chatbot behavior, and personalization must adapt to authenticated user profiles and stated backgrounds.
- Reproducibility: the full system (book, chatbot, backend services) must be reproducible from the repository using documented steps.

Primary Deliverables:
1. AI-Written Book
   - Authored using Claude Code and Spec-Kit Plus.
   - Structured and rendered using Docusaurus.
   - Deployed to GitHub Pages.
   - Written for a technical audience with software engineering and AI background.
   - Clear chapter hierarchy with navigation, code examples, and diagrams where appropriate.

2. Integrated RAG Chatbot
   - Embedded directly within the published book UI.
   - Built using:
     - OpenAI Agents and/or ChatKit SDKs
     - FastAPI backend
     - Neon Serverless Postgres (metadata, users, personalization)
     - Qdrant Cloud (Free Tier) for vector storage
   - Capabilities:
     - Answer questions about the entire book.
     - Answer questions based strictly on user-selected text.
     - Cite retrieved chunks internally (no hallucinated content).
     - Respect user authentication and personalization context.

3. Reusable Intelligence (Bonus)
   - Claude Code Subagents and Agent Skills must be defined, versioned, and reused.
   - Subagents may include (but are not limited to):
     - Chapter authoring agent
     - Technical reviewer agent
     - RAG ingestion/indexing agent
     - Personalization agent
   - Subagents must demonstrate measurable reuse across chapters or system components.

4. Authentication and Personalization (Bonus)
   - Signup and Signin implemented using Better Auth.
   - During signup, collect structured information about:
     - Software background
     - Hardware background
     - Experience level
   - Store this data securely and use it to:
     - Personalize chapter explanations.
     - Adapt chatbot responses.

5. Chapter-Level Personalization (Bonus)
   - Each chapter must include a UI control to personalize content.
   - Personalization must:
     - Use the logged-in user profile.
     - Adjust depth, examples, or explanations accordingly.
     - Be reversible and transparent to the user.

6. Urdu Translation Support (Bonus)
   - Each chapter must include a UI control to translate content into Urdu.
   - Translation must:
     - Preserve technical accuracy.
     - Be consistent across chapters.
     - Integrate with personalization where applicable.

Standards and Constraints:
- All AI-generated content must be reviewed for factual correctness.
- No hardcoded secrets; all credentials via environment variables.
- Modular architecture: book, backend, RAG pipeline, and auth must be independently testable.
- Clear separation between content generation, retrieval, and response synthesis.
- Open-source compatible licensing for all dependencies.

Documentation Requirements:
- README with setup, deployment, and reproduction steps.
- Architecture diagrams (book pipeline, RAG flow, auth flow).
- Spec-Kit specifications for:
  - Book generation
  - RAG ingestion and querying
  - Authentication and personalization
- Clear explanation of how Claude Code and Subagents are used.

Success Criteria:
- Book successfully deployed and accessible via GitHub Pages.
- Emb

## Response snapshot

Updated the project constitution at .specify/memory/constitution.md with the specific details for the AI-Spec–Driven Interactive Book with Embedded RAG Chatbot project. The new constitution includes core principles of spec-driven development, AI-native authorship, technical accuracy, user-centric intelligence, and reproducibility. It also defines the primary deliverables, standards and constraints, documentation requirements, and success criteria as specified in the user input. The version was set to 1.0.0 with today's date as both ratification and last amended date.

## Outcome

- ✅ Impact: Successfully updated project constitution with all required principles and deliverables for the AI-Spec–Driven Interactive Book with Embedded RAG Chatbot project
- 🧪 Tests: constitution update verified
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: Implementation of primary deliverables based on updated constitution
- 🧠 Reflection: The constitution now serves as the authoritative guide for all development activities in this project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): Standard constitution update
- Next experiment (smallest change to try): Begin implementation of primary deliverables according to the established constitution