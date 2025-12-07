# Data Model: Physical AI & Humanoid Robotics Book

This document defines the data models for the project, focusing on the RAG chatbot and user-related features.

## RAG Chatbot

### `Document`
Represents a source document from the book's content that is fed into the vector database.

- **id** (string, primary key): Unique identifier for the document (e.g., the file path).
- **content** (string): The full text content of the document.
- **metadata** (object): Additional information about the document.
  - **source** (string): The source of the document (e.g., `docs/module1/chapter1.md`).
  - **chapter** (string): The chapter the document belongs to.

### `Chunk`
Represents a chunk of a `Document` that is stored in the vector database.

- **id** (string, primary key): Unique identifier for the chunk.
- **document_id** (string, foreign key): The ID of the `Document` this chunk belongs to.
- **content** (string): The text content of the chunk.
- **embedding** (array of floats): The vector embedding of the content.

## User Personalization

### `User`
Represents a user of the book website, primarily for personalization features.

- **id** (string, primary key): Unique identifier for the user (e.g., a session ID or a persistent ID if we add authentication).
- **preferences** (object): User-specific preferences.
  - **level** (string): The user's preferred learning level ("beginner" or "advanced").

*Note: For the initial implementation, we will use a client-side solution (e.g., localStorage) to store user preferences, so a `User` model on the backend might not be necessary. This can be revisited if we add full user accounts.*
