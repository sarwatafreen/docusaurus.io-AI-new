# Quickstart: Physical AI & Humanoid Robotics Book

This guide provides the steps to set up the development environment for the "Physical AI & Humanoid Robotics" book.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed on your machine.
- [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) for cloning the repository.

## Setup

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/your-username/physical-ai-book.git
    cd physical-ai-book
    ```

2.  **Run the setup script**:
    This script will build the Docker container with all the necessary dependencies, including ROS 2, Gazebo, Isaac Sim, and the Python backend.
    ```bash
    ./start.sh
    ```

3.  **Access the development environment**:
    Once the script is finished, you will have a running Docker container. You can access it with:
    ```bash
    docker exec -it physical-ai-book-container bash
    ```

## Development

### Frontend (Docusaurus)

The Docusaurus development server will be running automatically. You can access the book website at [http://localhost:3000](http://localhost:3000).

To make changes to the frontend, edit the files in the `/frontend` directory. The development server will automatically reload.

### Backend (FastAPI)

The FastAPI backend for the RAG chatbot will also be running automatically. You can access the API documentation at [http://localhost:8000/docs](http://localhost:8000/docs).

To make changes to the backend, edit the files in the `/backend` directory. The development server will automatically reload.
