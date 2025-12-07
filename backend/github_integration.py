import os
import requests
from typing import List, Dict, Any, Optional
from pathlib import Path
import base64
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class GitHubRepoFetcher:
    """
    A class to fetch content from a GitHub repository for the RAG system.
    """
    
    def __init__(self, owner: str, repo: str):
        self.owner = owner
        self.repo = repo
        self.base_url = "https://api.github.com"
        
        # Use GitHub token if available for higher rate limits
        self.token = os.getenv("GITHUB_TOKEN")
        self.headers = {
            "Accept": "application/vnd.github.v3+json"
        }
        
        if self.token:
            self.headers["Authorization"] = f"token {self.token}"
    
    def _make_request(self, url: str) -> Dict[str, Any]:
        """
        Make a request to the GitHub API.
        """
        response = requests.get(url, headers=self.headers)
        response.raise_for_status()
        return response.json()
    
    def get_repo_contents(self, path: str = "") -> List[Dict[str, Any]]:
        """
        Get the contents of a repository path.
        """
        url = f"{self.base_url}/repos/{self.owner}/{self.repo}/contents/{path}"
        return self._make_request(url)
    
    def get_file_content(self, path: str) -> str:
        """
        Get the content of a specific file from the repository.
        """
        url = f"{self.base_url}/repos/{self.owner}/{self.repo}/contents/{path}"
        response = self._make_request(url)
        
        # GitHub API returns content as base64 encoded
        content = base64.b64decode(response['content']).decode('utf-8')
        return content
    
    def get_directory_tree(self, recursive: bool = False) -> Dict[str, Any]:
        """
        Get the git tree for the repository to list all files.
        """
        url = f"{self.base_url}/repos/{self.owner}/{self.repo}/git/trees/main"
        if recursive:
            url += "?recursive=1"
        
        try:
            return self._make_request(url)
        except requests.exceptions.HTTPError:
            # Try with 'master' branch if 'main' doesn't exist
            url = f"{self.base_url}/repos/{self.owner}/{self.repo}/git/trees/master"
            if recursive:
                url += "?recursive=1"
            return self._make_request(url)
    
    def download_repo_content(self, local_path: str = "./repo_content", file_extensions: List[str] = None) -> int:
        """
        Download content from the GitHub repository to a local directory.
        """
        if file_extensions is None:
            # Default extensions for documentation/content
            file_extensions = ['.md', '.txt', '.py', '.js', '.ts', '.jsx', '.tsx', '.html', '.css']
        
        # Create local directory if it doesn't exist
        Path(local_path).mkdir(parents=True, exist_ok=True)
        
        # Get the full tree to list all files
        tree_data = self.get_directory_tree(recursive=True)
        files = tree_data.get('tree', [])
        
        downloaded_count = 0
        
        for file in files:
            if file['type'] == 'blob':  # Only process files, not directories
                file_path = file['path']
                file_ext = Path(file_path).suffix.lower()
                
                # Only download files with specified extensions
                if file_extensions and file_ext not in file_extensions:
                    continue
                
                try:
                    # Create local file path
                    local_file_path = Path(local_path) / file_path
                    
                    # Create subdirectories if needed
                    local_file_path.parent.mkdir(parents=True, exist_ok=True)
                    
                    # Get file content from GitHub
                    content = self.get_file_content(file_path)
                    
                    # Write content to local file
                    with open(local_file_path, 'w', encoding='utf-8') as f:
                        f.write(content)
                    
                    downloaded_count += 1
                    logger.info(f"Downloaded: {file_path}")
                    
                except Exception as e:
                    logger.error(f"Error downloading {file_path}: {str(e)}")
        
        logger.info(f"Downloaded {downloaded_count} files from {self.owner}/{self.repo}")
        return downloaded_count
    
    def get_repo_info(self) -> Dict[str, Any]:
        """
        Get general information about the repository.
        """
        url = f"{self.base_url}/repos/{self.owner}/{self.repo}"
        return self._make_request(url)


class GitHubRAGIntegrator:
    """
    Integrates GitHub repository content with the RAG system.
    """
    
    def __init__(self, owner: str, repo: str, rag_service):
        self.fetcher = GitHubRepoFetcher(owner, repo)
        self.rag_service = rag_service
    
    def index_repo_content(self, file_extensions: List[str] = None) -> int:
        """
        Fetch content from GitHub repository and index it in the RAG system.
        """
        # Download repo content locally first
        download_count = self.fetcher.download_repo_content(
            local_path="./repo_content", 
            file_extensions=file_extensions
        )
        
        if download_count == 0:
            logger.warning("No files were downloaded from the repository")
            return 0
        
        # Now process the downloaded content with the document processor
        from document_processor import DocumentProcessor
        
        processor = DocumentProcessor()
        chunks = processor.process_directory("./repo_content", recursive=True)
        
        # Add all chunks to the RAG system
        doc_ids = self.rag_service.vector_store.add_documents(chunks)
        
        logger.info(f"Indexed {len(chunks)} content chunks from GitHub repository")
        return len(chunks)


# Example usage function
def main():
    # Example integration with sarwatafreen's docusaurus.io-AI repository
    fetcher = GitHubRepoFetcher("sarwatafreen", "docusaurus.io-AI")
    
    try:
        # Get repository information
        repo_info = fetcher.get_repo_info()
        print(f"Repository: {repo_info['full_name']}")
        print(f"Description: {repo_info['description']}")
        print(f"Stars: {repo_info['stargazers_count']}")
        
        # Download repository content
        fetcher.download_repo_content(file_extensions=['.md', '.txt', '.py', '.js', '.jsx', '.tsx', '.ts', '.html', '.css'])
        print("Repository content downloaded successfully")
        
    except Exception as e:
        print(f"Error: {str(e)}")


if __name__ == "__main__":
    main()