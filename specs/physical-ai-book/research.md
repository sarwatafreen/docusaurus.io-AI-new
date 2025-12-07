# Research: Physical AI & Humanoid Robotics Book

This document outlines the research required to resolve the "NEEDS CLARIFICATION" items from the implementation plan.

## 1. Storage: Vector Database for RAG

**Decision**: ChromaDB
**Rationale**: 
- ChromaDB is lightweight, open-source, and easy to set up, which aligns with the project's "one-click" philosophy.
- It can be run in-memory or as a separate Docker container, providing flexibility for development and deployment.
- It has good integration with LangChain, which is a core dependency for the RAG implementation.
**Alternatives considered**:
- **FAISS**: While powerful, it is more complex to set up and manage.
- **Pinecone**: A managed service, which goes against the goal of providing a self-contained, open-source project that can be run locally without external dependencies (and costs).

## 2. Testing Strategy

### Backend (FastAPI)
**Decision**: Pytest
**Rationale**:
- Pytest is the standard for testing in the Python community.
- It has a rich plugin ecosystem and is easy to use.
- FastAPI documentation and community resources heavily favor Pytest.
**Testing types**:
- **Unit tests**: For individual functions and classes.
- **Integration tests**: For testing the interaction between different services.
- **API tests**: For testing the API endpoints.

### Frontend (Docusaurus/React)
**Decision**: Jest and React Testing Library
**Rationale**:
- Jest is the most popular testing framework for React applications.
- React Testing Library provides a user-centric way of testing components, which aligns with the project's focus on user experience.
**Testing types**:
- **Component tests**: For individual React components.
- **Integration tests**: For testing the interaction between different components and pages.
- **End-to-end tests**: We will not be using a dedicated end-to-end testing framework like Cypress or Playwright.
**Rationale**:
- The primary goal of this project is to be a learning resource with a simple, one-click setup. Adding an end-to-end testing framework would increase the complexity of the environment and the learning curve for beginners.
- The most critical user journeys (like the RAG chatbot and the final capstone project) will be manually tested.
- This decision can be revisited in the future if the project grows in complexity.
