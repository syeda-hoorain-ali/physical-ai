# Implementation Plan: Chatbot with OpenAI Agents

**Branch**: `005-chatbot-openai-agents` | **Date**: 2025-12-05 | **Spec**: /specs/005-chatbot-openai-agents/spec.md
**Input**: Feature specification from `/specs/005-chatbot-openai-agents/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a simple chatbot using Python backend with Python ChatKit server framework and OpenAI Agents SDK for creating one AI agent. The frontend will utilize OpenAI ChatKit UI components for a seamless chat experience. The system will handle real-time chat interactions with Neon database for chat persistence, basic content moderation and graceful error handling.

## Technical Context

**Language/Version**: Python 3.11+, TypeScript
**Primary Dependencies**: OpenAI Agents SDK, openai-chatkit, Pydantic, Neon database adapter, OpenAI ChatKit (React)
**Package Manager**: uv (modern, fast Python package manager written in Rust)
**Storage**: Neon PostgreSQL database for persistent chat storage
**Testing**: pytest for backend, React Testing Library for frontend
**Target Platform**: Web application (Linux/Windows/Mac compatible)
**Project Type**: Web application (backend + frontend)
**Performance Goals**: <10 second response time for AI interactions
**Constraints**: OpenAI API rate limits, security for user inputs
**Scale/Scope**: Simple chatbot with single agent, persistent storage with Neon database

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[Gates determined based on constitution file]

## Project Structure

### Documentation (this feature)

```text
specs/005-chatbot-openai-agents/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Python ChatKit server entry point
├── server.py            # ChatKit server implementation extending ChatKitServer
├── agent.py             # Single OpenAI agent implementation
├── stores.py            # Data and attachment store implementations for Neon database
├── pyproject.toml       # Project metadata and dependencies for uv
├── uv.lock              # Lock file for deterministic builds
└── .venv                # Virtual environment managed by uv

book-source/ (Docusaurus documentation site with chat widget)
├── docusaurus.config.ts # Docusaurus configuration
├── package.json         # Frontend dependencies
├── src/
│   ├── components/      # React components
│   │   └── chat-widget/  # Chat widget component (circle icon that opens chat)
│   │       ├── chat-widget.tsx
│   │       └── chat-modal.tsx
│   ├── pages/           # Additional pages if needed
│   ├── theme/
│   │   └── Root.tsx     # Root component to add chat widget to every page
│   └── css/             # Custom styles
├── static/              # Static assets
└── docs/                # Documentation files
```

**Structure Decision**: Web application with Python ChatKit server backend and Docusaurus frontend documentation site. The Python ChatKit server handles chat interactions using the OpenAI Agents SDK with Neon database for persistent storage. The chat widget is implemented as a persistent component using the Root.tsx approach, ensuring it appears on every page as a circle icon that opens a chat window when clicked. Uses uv for fast, modern Python package management for the backend.
