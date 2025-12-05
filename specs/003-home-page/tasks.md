---
description: "Task list for Home Page for Physical AI & Humanoid Robots Textbook implementation"
---

# Tasks: Home Page for Physical AI & Humanoid Robots Textbook

**Input**: Design documents from `/specs/003-home-page/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md

**Tests**: The feature specification does not explicitly request writing tests *before* implementation, but the plan includes testing frameworks. Test tasks will be included to be written *after* the implementation for each user story, and also cross-cutting E2E tests.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
-   Include exact file paths in descriptions

## Path Conventions

-   Paths shown assume Docusaurus structure as defined in `plan.md`.

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, theme configuration, and setting up core Docusaurus for development.

-   [x] T001 Initialize Docusaurus project if not already done. (Assuming `book-source/` already exists and is a Docusaurus project, this might be updating dependencies or checking config)
-   [x] T002 Configure Tailwind CSS in `book-source/tailwind.config.ts` and `book-source/src/css/global.css`.
-   [x] T003 [P] Configure Docusaurus `docusaurus.config.ts` for global CSS and potential plugins.
-   [x] T004 Install Magic UI components and any necessary dependencies in `book-source/`.
-   [x] T005 Setup theme (aqua marine & chartreuse green) in `book-source/src/css/global.css` and `book-source/docusaurus.config.ts`. (FR-001)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: No specific foundational tasks beyond Docusaurus setup which is covered in Phase 1. This section is noted as "N/A" as no explicit blocking prerequisites are identified.

N/A

---

## Phase 3: User Story 1 - View Home Page (Priority: P1) 🎯 MVP

**Goal**: As a prospective reader, I want to view the textbook's home page to get an overview of the content and quickly navigate to start reading.

**Independent Test**: Navigate to the home page URL and observe all primary elements (navbar, main heading, introductory text, "start reading" button, first page content) are displayed correctly and match the theme.

### Implementation for User Story 1

-   [x] T006 [P] [US1] Create the basic `index.tsx` page structure in `book-source/src/pages/index.tsx`.
-   [x] T007 [P] [US1] Implement Navbar with "Interactive Hover Button" (label "read book") in `book-source/src/components/navbar.tsx` and integrate into `index.tsx`. (FR-002)
-   [x] T008 [P] [US1] Implement Hero section with "Light Rays" component in `book-source/src/components/home-page-sections/hero-section.tsx` and integrate into `index.tsx`. (FR-003 partial, reqs 13, 14, 15 from spec)
-   [x] T009 [P] [US1] Integrate "Aurora Text" and "Sparkles Text" for the main heading within the Hero section in `book-source/src/components/home-page-sections/hero-section.tsx`. (FR-003)
-   [x] T010 [P] [US1] Add a short, welcoming slogan and an "Interactive Hover Button" (label "start reading") below the main heading in `book-source/src/components/home-page-sections/hero-section.tsx`. (FR-004)
-   [x] T011 [P] [US1] Integrate "Scroll Based Velocity" component after the Hero section in `book-source/src/pages/index.tsx`. (FR-005)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Explore AI Content (Priority: P2)

**Goal**: As a curious reader, I want to see engaging content about AI, presented in an appealing card format and with dynamic scrolling messages on the second part of the home page.

**Independent Test**: Scroll down to the second part of the home page and verify the presence and functionality of the "Magic Card" and "Animated List" components, displaying relevant AI content.

### Implementation for User Story 2

-   [x] T012 [P] [US2] Create `FeaturesSection` component to display multiple "Magic Card" components with AI-related content in `book-source/src/components/home-page-sections/features-section.tsx` and integrate into `index.tsx`. (FR-006)
-   [x] T013 [P] [US2] Create `AnimatedListSection` component to display an "Animated List" with scrolling messages in `book-source/src/components/home-page-sections/animated-list-section.tsx` and integrate into `index.tsx`. (FR-007)

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Ensure overall quality, responsiveness, and adherence to non-functional requirements.

-   [x] T014 Review and refine theme application (aqua marine & chartreuse green) across all components for consistency in `book-source/src/css/global.css`. (SC-003)
-   [x] T015 Implement responsive design adjustments in `book-source/src/pages/index.tsx` and components to ensure graceful adaptation to mobile, tablet, and desktop screen sizes. (FR-008, SC-005)
-   [x] T016 Conduct a performance review to ensure the home page loads and renders all specified components within 3 seconds. Optimize assets and component rendering if necessary. (SC-001)
-   [x] T017 Verify all Magic UI components are integrated and function as intended. (SC-002)
-   [x] T018 Ensure "start reading" button is clearly visible and clickable. (SC-004)

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately
-   **Foundational (Phase 2)**: N/A (no explicit foundational tasks)
-   **User Stories (Phase 3+)**: Can start after Setup (Phase 1) completion. User Story 1 should be prioritized.
-   **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Setup (Phase 1). No dependencies on other stories.
-   **User Story 2 (P2)**: Can start after Setup (Phase 1). No direct dependency on US1, but builds upon the initial page structure.

### Within Each User Story

-   Implementation tasks should precede testing tasks.
-   Core component creation before integration into `index.tsx`.

### Parallel Opportunities

-   All Setup tasks marked [P] can run in parallel.
-   Within User Story 1, tasks T007, T008, T009, T010, T011 can be worked on in parallel once their preceding dependencies are met.
-   Within User Story 2, tasks T015, T016 can be worked on in parallel.
-   Different user stories could theoretically be worked on in parallel by different team members after the Setup phase.

---

## Parallel Example: User Story 1

```bash
# Example of parallel implementation for US1 components
# Assuming T006 (basic index.tsx) is complete
Task: "Implement Navbar with 'Interactive Hover Button' in book-source/src/components/navbar.tsx"
Task: "Implement Hero section with 'Light Rays' component in book-source/src/components/home-page-sections/hero-section.tsx"

# After main components are ready, integrate specific elements
Task: "Integrate 'Aurora Text' and 'Sparkles Text' for main heading within Hero section"
Task: "Add slogan and 'Interactive Hover Button' (label 'start reading') below main heading"
Task: "Integrate 'Scroll Based Velocity' component after Hero section in index.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 3: User Story 1 Implementation tasks
3.  Complete Phase 3: User Story 1 Test tasks
4.  **STOP and VALIDATE**: Test User Story 1 independently
5.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup → Foundation ready
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3.  Add User Story 2 → Test independently → Deploy/Demo
4.  Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup together.
2.  Once Setup is done:
    -   Developer A: User Story 1 (Implementation & Testing)
    -   Developer B: User Story 2 (Implementation & Testing)
3.  Stories complete and integrate independently.
4.  Finally, perform Phase 5: Polish & Cross-Cutting Concerns.

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing (if doing TDD, which is not explicitly requested here)
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
