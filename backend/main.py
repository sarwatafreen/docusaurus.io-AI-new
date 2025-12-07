from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from chatbot import get_gemini_response
from vector_store import RAGService
from document_processor import DocumentProcessor
import os
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Initialize services
rag_service = RAGService(collection_name="book_content")
doc_processor = DocumentProcessor()

app = FastAPI(title="RAG Chatbot API", description="A RAG chatbot API using Google's Gemini with book content retrieval", version="1.0.0")

# CORS middleware to allow requests from Docusaurus frontend (localhost:3000)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Add your Docusaurus frontend origin
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    message: str
    use_rag: bool = True  # Whether to use RAG or just the base model
    selected_text: str = None  # Optional selected text from the book

class IndexRequest(BaseModel):
    content: str
    metadata: dict = {}

@app.get("/")
def read_root():
    return {"message": "Welcome to the RAG Chatbot API"}

@app.post("/chat")
async def chat_endpoint(chat_request: ChatRequest):
    try:
        user_message = chat_request.message

        # If selected_text is provided, use it as context regardless of use_rag flag
        if chat_request.selected_text:
            # Create a prompt that includes the selected text as context
            full_prompt = f"Based on the following information, answer the question: '{user_message}'\n\nSelected Text:\n{chat_request.selected_text}"
            response = get_gemini_response(full_prompt)
        elif chat_request.use_rag:
            # Use RAG to get context from book content
            search_results = rag_service.query(user_message, top_k=3)

            if search_results:
                # Combine the relevant passages as context
                context_parts = [result['content'] for result in search_results]
                context = "\n\n".join(context_parts)

                # Create a prompt that includes the context
                full_prompt = f"Based on the following information, answer the question: '{user_message}'\n\nContext:\n{context}"
                response = get_gemini_response(full_prompt)
            else:
                # If no relevant content found, respond without context
                response = get_gemini_response(user_message)
        else:
            # Just use the base model without RAG
            response = get_gemini_response(user_message)

        return {
            "reply": response,
            "use_rag": chat_request.use_rag,
            "selected_text_used": bool(chat_request.selected_text)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing request: {str(e)}")

@app.post("/index")
async def index_content(index_request: IndexRequest):
    """Endpoint to add content to the RAG system"""
    try:
        # Add content to the RAG system
        doc_id = rag_service.add_document_content(
            content=index_request.content,
            metadata=index_request.metadata
        )

        return {"message": "Content indexed successfully", "doc_id": doc_id}
    except Exception as e:
        logging.error(f"Error indexing content: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error indexing content: {str(e)}")

@app.post("/index-file")
async def index_file(file_path: str):
    """Endpoint to process and index an entire file"""
    try:
        # Process the file using DocumentProcessor
        chunks = doc_processor.process_file(file_path)

        # Add all chunks to the RAG system
        doc_ids = rag_service.vector_store.add_documents(chunks)

        return {
            "message": f"File indexed successfully with {len(doc_ids)} chunks",
            "doc_ids": doc_ids
        }
    except Exception as e:
        logging.error(f"Error indexing file: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error indexing file: {str(e)}")

@app.get("/health")
def health_check():
    """Health check endpoint"""
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)