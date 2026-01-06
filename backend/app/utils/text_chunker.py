from typing import List, Dict, Any
from config import Config


class TextChunker:
    """
    Utility class for chunking text content into smaller pieces for embedding
    """

    def __init__(self, chunk_size: int = None, overlap_size: int = None):
        self.chunk_size = chunk_size or Config.CHUNK_SIZE
        self.overlap_size = overlap_size or Config.OVERLAP_SIZE

    def chunk_content(self, content: str, metadata: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Chunk the content into smaller pieces with overlap
        """
        if not content or len(content.strip()) == 0:
            return []

        # Split content into sentences to avoid breaking in the middle of sentences
        sentences = self._split_into_sentences(content)
        chunks = []
        current_chunk = ""
        current_start_idx = 0

        for sentence in sentences:
            # Check if adding this sentence would exceed chunk size
            if len(current_chunk + sentence) > self.chunk_size:
                # If the current chunk is not empty, save it
                if current_chunk.strip():
                    chunk_data = {
                        'content': current_chunk.strip(),
                        'metadata': metadata or {},
                        'start_idx': current_start_idx,
                        'end_idx': current_start_idx + len(current_chunk)
                    }
                    chunks.append(chunk_data)

                # Start a new chunk with overlap
                if len(sentence) > self.chunk_size:
                    # If sentence is longer than chunk size, split it by chunk size
                    sub_chunks = self._split_long_sentence(sentence)
                    for i, sub_chunk in enumerate(sub_chunks):
                        if i == 0:
                            # First sub-chunk: just add it
                            current_chunk = sub_chunk
                            current_start_idx = current_start_idx + len(current_chunk) - len(sub_chunk)
                        else:
                            # For subsequent sub-chunks, add overlap from the previous chunk
                            overlap = current_chunk[-self.overlap_size:] if len(current_chunk) >= self.overlap_size else current_chunk
                            current_chunk = overlap + sub_chunk
                            current_start_idx = current_start_idx + len(overlap)

                        if len(current_chunk) >= self.chunk_size:
                            chunk_data = {
                                'content': current_chunk.strip(),
                                'metadata': metadata or {},
                                'start_idx': current_start_idx,
                                'end_idx': current_start_idx + len(current_chunk)
                            }
                            chunks.append(chunk_data)
                            current_chunk = current_chunk[-self.overlap_size:] if len(current_chunk) >= self.overlap_size else current_chunk
                            current_start_idx = current_start_idx + len(current_chunk) - len(current_chunk[-self.overlap_size:] if len(current_chunk) >= self.overlap_size else current_chunk)
                else:
                    # Start new chunk with overlap from the end of the previous chunk
                    overlap = current_chunk[-self.overlap_size:] if len(current_chunk) >= self.overlap_size else current_chunk
                    current_chunk = overlap + sentence
                    current_start_idx = current_start_idx + len(current_chunk) - len(overlap) - len(sentence)
            else:
                # Add sentence to current chunk
                current_chunk += sentence

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunk_data = {
                'content': current_chunk.strip(),
                'metadata': metadata or {},
                'start_idx': current_start_idx,
                'end_idx': current_start_idx + len(current_chunk)
            }
            chunks.append(chunk_data)

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences
        """
        import re
        # Split on sentence endings followed by whitespace and capital letter or end of string
        sentences = re.split(r'(?<=[.!?])\s+', text)
        # Add back the sentence endings
        sentences = [s + '.' if not s.endswith(('.', '!', '?')) and i < len(sentences) - 1 else s
                     for i, s in enumerate(sentences)]
        return sentences

    def _split_long_sentence(self, sentence: str) -> List[str]:
        """
        Split a sentence that is longer than the chunk size
        """
        if len(sentence) <= self.chunk_size:
            return [sentence]

        chunks = []
        start = 0
        while start < len(sentence):
            end = start + self.chunk_size
            if end >= len(sentence):
                chunks.append(sentence[start:])
                break
            else:
                chunks.append(sentence[start:end])
                start = end - self.overlap_size  # Apply overlap

        return chunks

    def chunk_multiple_contents(self, contents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Chunk multiple content items with their metadata
        """
        all_chunks = []
        for item in contents:
            content = item.get('content', '')
            metadata = {k: v for k, v in item.items() if k != 'content'}
            chunks = self.chunk_content(content, metadata)
            all_chunks.extend(chunks)
        return all_chunks