# Research Summary: User Authentication with Personalized Content

## Decision: Authentication Approach
**Rationale**: For a Docusaurus site hosted on GitHub Pages (static site), we need an external authentication service. Better Auth provides the flexibility to run the auth service separately while integrating with the frontend via API calls.

## Decision: Database Choice
**Rationale**: PostgreSQL was selected as it provides robust support for user data and background information while scaling well for educational platforms. It's also well-supported by Better Auth.

## Decision: Custom User Schema
**Rationale**: Better Auth supports custom user attributes which allows us to store software and hardware background information directly on user accounts. This simplifies the personalization logic.

## Decision: Frontend Integration
**Rationale**: Using Better Auth's React integration provides session management and authentication state handling that works well with Docusaurus's React-based architecture.

## Alternatives Considered:
1. Client-only auth solutions - rejected due to security concerns and limitations with static hosting
2. Third-party auth providers only - rejected as it doesn't allow collection of custom background information
3. Custom auth implementation - rejected due to complexity and security considerations
4. Different database options - PostgreSQL chosen for its reliability and feature set for user data

## Architecture Pattern:
- External Better Auth service (backend)
- Docusaurus frontend with React auth components
- PostgreSQL database for user accounts and background data
- API-based communication between frontend and auth service

## OpenAI ChatKit
- Batteries-included framework for building high-quality, AI-powered chat experiences
- Provides deep UI customization and built-in response streaming
- Includes React bindings (@openai/chatkit-react) for easy integration
- Requires backend API to generate client secrets for session management
- Supports entity tagging, client-side tools, and multi-thread conversations
- Backend (Python/ChatKit Server): Handle chat interactions, manage persistent sessions with Neon database, interact with OpenAI Agents SDK
- Frontend (Docusaurus/ChatKit): Provide persistent chat widget using OpenAI ChatKit components
