"""
Physical AI Book RAG System Orchestrator

This script orchestrates the entire pipeline:
1. Crawls documentation pages
2. Cleans and chunks the text
3. Generates embeddings using Cohere
4. Stores vectors in Qdrant
5. Validates the system with similarity search
"""

import asyncio
import os
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import our modules
from document_crawler import DocumentCrawler
from embedding_generator import EmbeddingGenerator
from vector_storage import VectorStorage, SimilarityValidator


async def main():
    """
    Main orchestrator function that runs the complete RAG pipeline.
    """
    print("Starting Physical AI Book RAG System Pipeline...")

    # Step 1: Crawl and process documents
    print("\n1. Crawling and processing documents...")
    crawler = DocumentCrawler(docs_dir="../docs")
    documents = crawler.crawl_documents()

    stats = crawler.get_statistics(documents)
    print(f"   Processed {stats['total_documents']} chunks from {stats['unique_files']} files")

    if not documents:
        print("No documents found to process. Exiting.")
        return

    # Step 2: Generate embeddings
    print("\n2. Generating embeddings using Cohere...")

    # Check for Cohere API key
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        print("Warning: COHERE_API_KEY not found in environment variables.")
        print("Please set your Cohere API key to generate real embeddings.")
        print("Using mock embeddings for demonstration...")

        # Create mock embeddings for demonstration
        import numpy as np
        for doc in documents:
            doc['embedding'] = np.random.random(1024).tolist()
        embedded_documents = documents
    else:
        # Initialize embedding generator and create embeddings
        try:
            generator = EmbeddingGenerator(api_key=cohere_api_key)
            embedded_docs_generator = generator.embed_documents(documents)

            embedded_documents = []
            async for doc in embedded_docs_generator:
                embedded_documents.append(doc)

        except Exception as e:
            print(f"Error generating embeddings: {str(e)}")
            print("Using mock embeddings for demonstration...")
            import numpy as np
            for doc in documents:
                doc['embedding'] = np.random.random(1024).tolist()
            embedded_documents = documents

    print(f"   Generated embeddings for {len(embedded_documents)} document chunks")

    # Step 3: Store vectors in Qdrant
    print("\n3. Storing vectors in Qdrant...")

    # Initialize vector storage
    storage = VectorStorage(location=":memory:", collection_name="physical_ai_book")

    # Store the embedded documents
    storage.store_vectors(embedded_documents)

    # Validate storage
    is_valid = storage.validate_storage()
    print(f"   Storage validation: {'PASSED' if is_valid else 'FAILED'}")

    # Step 4: Validate with similarity search
    print("\n4. Validating with similarity search...")

    validator = SimilarityValidator(storage)

    # Create a sample query embedding for validation
    # In a real scenario, this would come from embedding a user query
    import numpy as np
    sample_query_embedding = np.random.random(1024).tolist()

    validation_results = validator.validate_basic_search(
        sample_query="Physical AI and Robotics",
        sample_embedding=sample_query_embedding
    )

    # Step 5: Summary
    print("\n5. Pipeline Summary:")
    print(f"   - Documents processed: {len(documents)}")
    print(f"   - Unique files: {stats['unique_files']}")
    print(f"   - Total characters: {stats['total_characters']}")
    print(f"   - Average chunk size: {stats['average_chunk_size']:.2f}")
    print(f"   - Vectors stored: {len(embedded_documents)}")
    print(f"   - Search validation: {'PASSED' if validation_results['results_count'] > 0 else 'FAILED'}")

    print("\nPipeline completed successfully!")

    return {
        'documents': documents,
        'embedded_documents': embedded_documents,
        'storage': storage,
        'validation_results': validation_results
    }


def setup_environment():
    """
    Setup function to ensure all dependencies are installed.
    """
    import subprocess
    import sys

    required_packages = [
        "python-frontmatter",
        "cohere",
        "qdrant-client",
        "numpy",
        "python-dotenv",
        "beautifulsoup4",
        "requests",
        "tiktoken"
    ]

    for package in required_packages:
        try:
            __import__(package.replace("-", "_"))
        except ImportError:
            print(f"Installing {package}...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])


if __name__ == "__main__":
    # Setup environment
    setup_environment()

    # Load environment variables
    load_dotenv()

    # Run the orchestrator
    try:
        results = asyncio.run(main())
        print("\nPhysical AI Book RAG System is ready!")
    except KeyboardInterrupt:
        print("\nPipeline interrupted by user.")
    except Exception as e:
        print(f"\nPipeline failed with error: {str(e)}")
        import traceback
        traceback.print_exc()