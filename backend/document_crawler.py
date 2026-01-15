"""
Document Crawler for Physical AI Book

This script crawls through the documentation files, extracts text content,
cleans it, and splits it into semantic chunks for further processing.
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Tuple
import frontmatter  # For parsing markdown with frontmatter


class DocumentCrawler:
    def __init__(self, docs_dir: str = "../docs"):
        self.docs_dir = Path(docs_dir)
        self.chunks = []

    def clean_text(self, text: str) -> str:
        """
        Clean extracted text by removing markdown syntax and cleaning up whitespace.
        """
        # Remove markdown headers but preserve the content
        text = re.sub(r'^#+\s+', '', text, flags=re.MULTILINE)

        # Remove bold and italic markdown syntax
        text = re.sub(r'\*\*(.*?)\*\*', r'\1', text)
        text = re.sub(r'\*(.*?)\*', r'\1', text)
        text = re.sub(r'__(.*?)__', r'\1', text)
        text = re.sub(r'_(.*?)_', r'\1', text)

        # Remove inline code markers
        text = re.sub(r'`(.*?)`', r'\1', text)

        # Remove image and link syntax, keeping the text
        text = re.sub(r'!\[(.*?)\]\(.*?\)', r'\1', text)  # Images
        text = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', text)   # Links

        # Remove blockquotes
        text = re.sub(r'^>\s+', '', text, flags=re.MULTILINE)

        # Remove horizontal rules
        text = re.sub(r'^[-*_]{3,}$', '', text, flags=re.MULTILINE)

        # Clean up extra whitespace
        text = re.sub(r'\n\s*\n', '\n\n', text)  # Multiple blank lines to single
        text = re.sub(r'[ \t]+', ' ', text)      # Multiple spaces to single
        text = text.strip()

        return text

    def split_into_chunks(self, text: str, max_chunk_size: int = 1000, overlap: int = 100) -> List[str]:
        """
        Split text into semantic chunks with overlap.
        """
        paragraphs = text.split('\n\n')
        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            paragraph = paragraph.strip()
            if not paragraph:
                continue

            # If adding this paragraph would exceed the chunk size
            if len(current_chunk) + len(paragraph) > max_chunk_size and current_chunk:
                chunks.append(current_chunk.strip())

                # Add overlap by taking the last part of the current chunk
                if overlap > 0:
                    overlap_text = current_chunk[-overlap:] if len(current_chunk) > overlap else current_chunk
                    current_chunk = overlap_text + " " + paragraph
                else:
                    current_chunk = paragraph
            else:
                current_chunk += "\n\n" + paragraph if current_chunk else paragraph

                # If current chunk is getting large, save it
                if len(current_chunk) >= max_chunk_size:
                    chunks.append(current_chunk.strip())
                    current_chunk = ""

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def extract_content_from_file(self, file_path: Path) -> Tuple[str, Dict]:
        """
        Extract content from a markdown file, removing frontmatter.
        """
        post = frontmatter.load(file_path)
        content = post.content
        metadata = post.metadata

        # Add file-specific metadata
        metadata['file_path'] = str(file_path.relative_to(self.docs_dir))
        metadata['filename'] = file_path.name
        metadata['title'] = metadata.get('title', file_path.stem)

        return content, metadata

    def crawl_documents(self) -> List[Dict]:
        """
        Crawl through all markdown documents and extract content.
        """
        markdown_files = list(self.docs_dir.rglob("*.md"))
        documents = []

        for md_file in markdown_files:
            try:
                content, metadata = self.extract_content_from_file(md_file)
                cleaned_content = self.clean_text(content)

                # Split into chunks
                chunks = self.split_into_chunks(cleaned_content)

                for i, chunk in enumerate(chunks):
                    document = {
                        'id': f"{md_file.stem}_chunk_{i}",
                        'content': chunk,
                        'metadata': {
                            **metadata,
                            'chunk_index': i,
                            'total_chunks': len(chunks),
                            'file_path': str(md_file.relative_to(self.docs_dir)),
                            'absolute_path': str(md_file.absolute())
                        }
                    }
                    documents.append(document)

            except Exception as e:
                print(f"Error processing {md_file}: {str(e)}")

        return documents

    def get_statistics(self, documents: List[Dict]) -> Dict:
        """
        Get statistics about the crawled documents.
        """
        total_docs = len(documents)
        total_chunks = len(documents)  # Each dict represents a chunk
        total_chars = sum(len(doc['content']) for doc in documents)
        avg_chunk_size = total_chars / total_chunks if total_chunks > 0 else 0

        # Get unique file paths
        unique_files = set(doc['metadata']['file_path'] for doc in documents)

        return {
            'total_documents': total_docs,
            'unique_files': len(unique_files),
            'total_characters': total_chars,
            'average_chunk_size': avg_chunk_size,
            'file_paths': sorted(list(unique_files))
        }


def main():
    """
    Main function to run the document crawler.
    """
    print("Starting document crawling process...")

    crawler = DocumentCrawler()
    documents = crawler.crawl_documents()

    stats = crawler.get_statistics(documents)

    print(f"Successfully processed {stats['total_documents']} chunks from {stats['unique_files']} files")
    print(f"Total characters: {stats['total_characters']}")
    print(f"Average chunk size: {stats['average_chunk_size']:.2f}")

    # Show a sample of the first few chunks
    print("\nSample chunks:")
    for i, doc in enumerate(documents[:3]):
        print(f"\nChunk {i+1}:")
        print(f"  File: {doc['metadata']['file_path']}")
        print(f"  Chunk: {doc['metadata']['chunk_index']+1}/{doc['metadata']['total_chunks']}")
        print(f"  Length: {len(doc['content'])} characters")
        print(f"  Content preview: {doc['content'][:100]}...")

    return documents


if __name__ == "__main__":
    # Install required dependency first
    import subprocess
    import sys

    try:
        import frontmatter
    except ImportError:
        print("Installing python-frontmatter...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "python-frontmatter"])
        import frontmatter

    documents = main()