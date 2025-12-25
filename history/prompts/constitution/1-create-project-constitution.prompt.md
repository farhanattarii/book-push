---
id: 1
title: Create-Project-Constitution 
stage: constitution
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: dell5490
command: /sp.constitution
labels: [constitution, spec-driven-development, ai-book]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

Project: AI/Spec-Driven Book with Embedded RAG Chatbot

Core principles:

Spec-first workflow using Spec-Kit Plus
Technical accuracy from official sources
Clear, developer-focused writing
Reproducible setup and deployment

Key standards:
Book written with Docusaurus and deployed on GitHub Pages
RAG chatbot grounded only in book content or user-selected text
Stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud
Runnable, well-documented code

Constraints:
GitHub-based source control
No hallucinated responses
End-to-end reproducibility

Success criteria:
Live book on GitHub Pages
Fully functional embedded RAG chatbot
All specs implemented via Spec-Kit Plus

## Response snapshot

Updated the project constitution to reflect the specific requirements for the AI/Spec-Driven Book with Embedded RAG Chatbot project. The constitution now includes six core principles:

1. Spec-First Workflow
2. Technical Accuracy from Official Sources
3. Clear, Developer-Focused Writing
4. Reproducible Setup and Deployment
5. Quality Standards and Documentation
6. No Hallucination Policy

Also added sections for Technology Stack and Standards, and Development Workflow and Success Criteria. The version was updated from 1.0.0 to 1.1.0 to reflect the addition of new principles and content.

## Outcome

- ✅ Impact: Updated project constitution with specific principles and guidelines for the AI/Spec-Driven Book project
- 🧪 Tests: none
- 📁 Files: 1 modified (.specify/memory/constitution.md)
- 🔁 Next prompts: none
- 🧠 Reflection: Constitution now accurately reflects the project requirements and constraints

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A