# Physical AI Book Backend

This backend system implements a Retrieval-Augmented Generation (RAG) pipeline for the Physical AI & Humanoid Robotics book.

## Components

### 1. Document Crawler (`document_crawler.py`)
- Crawls through all Markdown files in the `../docs` directory
- Extracts content while preserving metadata
- Cleans text by removing Markdown syntax
- Splits content into semantic chunks with overlap

### 2. Embedding Generator (`embedding_generator.py`)
- Generates embeddings using Cohere's `embed-multilingual-v3.0` model
- Processes documents in batches respecting API limits
- Optimized for search_document use case

### 3. Vector Storage (`vector_storage.py`)
- Stores embeddings in Qdrant vector database
- Maintains document metadata alongside vectors
- Implements similarity search functionality
- Validates storage integrity

### 4. RAG Orchestrator (`rag_orchestrator.py`)
- Coordinates the entire pipeline
- Handles document processing, embedding, and storage
- Performs validation with similarity search

## Setup

1. Install dependencies:
```bash
uv pip install -r requirements.txt
```

2. Set up environment variables:
```bash
cp .env.example .env
# Add your Cohere API key to .env
```

3. Run the complete pipeline:
```bash
python rag_orchestrator.py
```

## Environment Variables

- `COHERE_API_KEY`: Your Cohere API key for generating embeddings

## Architecture

The system follows a modular design where each component handles a specific aspect of the RAG pipeline:

```
[Docs] → [Crawler] → [Cleaner] → [Chunker] → [Embedder] → [Qdrant Storage] → [Search]
```

## Features

- Automatic document discovery and processing
- Semantic chunking with configurable sizes
- Batch processing for efficient API usage
- In-memory or persistent Qdrant storage
- Comprehensive validation at each step
- Detailed logging and statistics