import os
import uuid
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class VectorStore:
    """
    A class to handle embedding generation and vector storage using Qdrant.
    """
    
    def __init__(self, collection_name: str = "book_content", model_name: str = "all-MiniLM-L6-v2"):
        # Initialize the embedding model
        self.model = SentenceTransformer(model_name)
        
        # Connect to Qdrant (using local instance by default)
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY", None)
        
        if qdrant_api_key:
            self.client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            # If no API key, try to connect without authentication (for local instance)
            self.client = QdrantClient(url=qdrant_url, prefer_grpc=False)
        
        self.collection_name = collection_name
        
        # Create the collection if it doesn't exist
        self._create_collection()
    
    def _create_collection(self):
        """
        Create a Qdrant collection for storing document embeddings.
        """
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' already exists")
        except:
            # If collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.model.get_sentence_embedding_dimension(),
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection '{self.collection_name}'")
    
    def add_documents(self, documents: List[Dict[str, Any]]) -> List[str]:
        """
        Add documents to the vector store and return their IDs.
        Each document should have 'content' and 'metadata' keys.
        """
        if not documents:
            logger.info("No documents to add")
            return []
        
        # Extract content for embedding
        contents = [doc['content'] for doc in documents]
        
        # Generate embeddings
        embeddings = self.model.encode(contents)
        
        # Prepare points for insertion
        points = []
        inserted_ids = []
        
        for i, (doc, embedding) in enumerate(zip(documents, embeddings)):
            doc_id = str(uuid.uuid4())
            inserted_ids.append(doc_id)
            
            points.append(
                models.PointStruct(
                    id=doc_id,
                    vector=embedding.tolist(),
                    payload={
                        "content": doc['content'],
                        "metadata": doc['metadata']
                    }
                )
            )
        
        # Insert points to the collection
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        
        logger.info(f"Added {len(documents)} documents to collection '{self.collection_name}'")
        return inserted_ids
    
    def search(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar documents to the query.
        Returns a list of documents with their content and metadata.
        """
        # Generate embedding for the query
        query_embedding = self.model.encode([query])[0].tolist()
        
        # Search in Qdrant
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k
        )
        
        # Format results
        results = []
        for result in search_results:
            results.append({
                'content': result.payload['content'],
                'metadata': result.payload['metadata'],
                'score': result.score
            })
        
        return results
    
    def delete_collection(self):
        """
        Delete the entire collection.
        """
        try:
            self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted collection '{self.collection_name}'")
        except Exception as e:
            logger.error(f"Error deleting collection '{self.collection_name}': {str(e)}")
    
    def get_collection_info(self):
        """
        Get information about the collection.
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                'name': collection_info.config.params.vectors_count,
                'vector_count': collection_info.points_count,
                'config': collection_info.config
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            return None


class RAGService:
    """
    Main RAG service that combines document processing and vector storage.
    """
    
    def __init__(self, collection_name: str = "book_content"):
        self.vector_store = VectorStore(collection_name=collection_name)
    
    def add_document_content(self, content: str, metadata: Optional[Dict[str, Any]] = None) -> str:
        """
        Add a single piece of content to the RAG system.
        """
        if metadata is None:
            metadata = {}
        
        document = {
            'content': content,
            'metadata': metadata
        }
        
        doc_ids = self.vector_store.add_documents([document])
        return doc_ids[0] if doc_ids else None
    
    def query(self, question: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Query the RAG system to find relevant documents for a question.
        """
        return self.vector_store.search(question, top_k=top_k)
    
    def query_and_respond(self, question: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Query the RAG system and format the response.
        """
        results = self.query(question, top_k)
        
        return {
            'question': question,
            'results': results,
            'retrieved_count': len(results)
        }


# Example usage function
def main():
    # Initialize RAG service
    rag_service = RAGService(collection_name="book_content")
    
    # Example: Add some sample content
    sample_content = "This is a sample content from the book to demonstrate RAG functionality."
    metadata = {"source": "sample", "type": "paragraph", "id": "1"}
    
    # Add content to the RAG system
    doc_id = rag_service.add_document_content(sample_content, metadata)
    print(f"Added document with ID: {doc_id}")
    
    # Query the RAG system
    question = "What is this sample content about?"
    response = rag_service.query_and_respond(question)
    print(f"Query: {question}")
    print(f"Response: {response}")


if __name__ == "__main__":
    main()