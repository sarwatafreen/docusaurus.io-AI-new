# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/physical-ai-book/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure (`/backend`, `/frontend`, `/docs`)
- [X] T002 Initialize Docusaurus v3 project in `/frontend`
- [X] T003 Initialize FastAPI project in `/backend`
- [X] T004 Create Dockerfile for the backend service
- [X] T005 Create Dockerfile for the frontend service
- [X] T006 Create `docker-compose.yml` to orchestrate the services
- [X] T007 Create `start.sh` script to build and run the Docker environment

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [X] T008 [P] Configure basic Docusaurus layout and theme in `frontend/docusaurus.config.js`
- [X] T009 [P] Set up ChromaDB vector store in the backend
- [X] T010 Implement document loading and text splitting service in `backend/src/services/documents.py`
- [X] T011 Implement embedding generation service using LangChain in `backend/src/services/embedding.py`
- [X] T012 Create a script to process the book content from `/docs` and load it into ChromaDB

---

## Phase 3: User Story 2 - Interactive Learning with RAG Chatbot (Priority: P1) 🎯 MVP

**Goal**: Allow users to highlight text and get a simpler explanation from a chatbot.
**Independent Test**: The chatbot UI appears on text selection and provides relevant, simplified explanations for content from the book.

### Implementation for User Story 2

- [X] T013 [P] [US2] Create the RAG chatbot API endpoint `/chat` in `backend/src/api/chat.py`
- [X] T014 [US2] Implement the core RAG chain logic in `backend/src/services/rag.py`
- [X] T015 [P] [US2] Create the chatbot UI component in `frontend/src/components/Chatbot/index.tsx`
- [X] T016 [P] [US2] Create an API service in the frontend to communicate with the backend API in `frontend/src/services/api.ts`
- [ ] T017 [US2] Implement the text selection detection and chatbot trigger logic in the frontend.

---

## Phase 4: User Story 4 - Content Personalization (Priority: P2)

**Goal**: Allow users to toggle the visibility of advanced content sections.
**Independent Test**: Clicking the "Personalize My Level" button hides or shows content marked as advanced.

### Implementation for User Story 4

- [ ] T018 [P] [US4] Create the "Personalize My Level" button component in `frontend/src/components/PersonalizeButton.js`
- [ ] T019 [P] [US4] Create a React context or state management solution for the user's selected level in `frontend/src/contexts/UserLevelContext.js`
- [ ] T020 [US4] Create a custom Docusaurus component or wrapper that conditionally renders content based on the user's selected level.

---

## Phase 5: User Story 5 - Urdu Translation (Priority: P3)

**Goal**: Provide a one-click option to translate the book content into Urdu.
**Independent Test**: Clicking the "Urdu" button translates the website content.

### Implementation for User Story 5

- [ ] T021 [P] [US5] Configure Docusaurus for i18n (internationalization) in `frontend/docusaurus.config.js`
- [ ] T022 [US5] Create the language dropdown component in the Docusaurus navbar.
- [ ] T023 [P] [US5] Add Urdu (`ur`) as a supported locale.
- [ ] T024 [US5] Create placeholder translation files for the book content in `frontend/i18n/ur/docusaurus-plugin-content-docs/current/`.

---

## Phase 6: Book Content and Capstone Project (User Story 3)

**Goal**: Write the book's content and implement the final capstone project.
**Independent Test**: The full book is available on the site, and the final capstone project is runnable and works as described in the spec.

- [ ] T025 Write Introduction chapter content in `/docs/intro/`
- [ ] T026 [P] [US3] Implement Chapter 1 (ROS 2 Node) code examples.
- [ ] T027 [P] [US3] Implement Chapter 2 (ROS 2 Topics) code examples.
- [ ] T028 [P] [US3] Implement Chapter 3 (ROS 2 Services & Actions) code examples.
- [ ] T029 [P] [US3] Implement Chapter 4 (URDF Mastery) code examples.
- [ ] T030 [P] [US3] Implement Chapter 5 (Gazebo Simulation) code examples.
- [ ] T031 [P] [US3] Implement Chapter 6 (Physics) code examples.
- [ ] T032 [P] [US3] Implement Chapter 7 (Sensors) code examples.
- [ ] T033 [P] [US3] Implement Chapter 8 (Unity Bonus) code examples.
- [ ] T034 [P] [US3] Implement Chapter 9 (Isaac Sim) code examples.
- [ ] T035 [P] [US3] Implement Chapter 10 (Data Generation) code examples.
- [ ] T036 [P] [US3] Implement Chapter 11 (Visual SLAM) code examples.
- [ ] T037 [P] [US3] Implement Chapter 12 (Nav2) code examples.
- [ ] T038 [P] [US3] Implement Chapter 13 (VLA - Whisper & LLM) code examples.
- [ ] T039 [P] [US3] Implement Chapter 14 (VLA - LangChain Planner) code examples.
- [ ] T040 [US3] Implement Final Capstone project.
- [ ] T041 Write Bonus Chapters content.

---

## Phase N: Polish & Cross-Cutting Concerns

- [ ] T042 [P] Set up GitHub Actions workflow for deploying the Docusaurus site to GitHub Pages in `.github/workflows/deploy.yml`
- [ ] T043 [P] Add tests for the backend API.
- [ ] T044 [P] Add tests for the frontend components.
- [ ] T045 Perform final manual testing of all features and user journeys.
- [ ] T046 Review and edit all book content for clarity and accuracy.
