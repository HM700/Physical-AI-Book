import os
import re
from pathlib import Path
from typing import List, Dict, Any
import markdown
from bs4 import BeautifulSoup
import yaml


class DocusaurusContentParser:
    """
    Parses Docusaurus markdown files to extract content with proper structure
    """

    def __init__(self, base_path: str):
        self.base_path = Path(base_path)

    def parse_all_markdown_files(self) -> List[Dict[str, Any]]:
        """
        Parse all markdown files in the Docusaurus structure
        Returns list of content chunks with metadata
        """
        content_chunks = []

        # Find all markdown files in the docs directory
        for md_file in self.base_path.rglob("*.md"):
            if md_file.name.startswith('_'):  # Skip files starting with underscore
                continue

            try:
                chunk_data = self._parse_markdown_file(md_file)
                content_chunks.extend(chunk_data)
            except Exception as e:
                print(f"Error parsing {md_file}: {str(e)}")

        return content_chunks

    def _parse_markdown_file(self, file_path: Path) -> List[Dict[str, Any]]:
        """
        Parse a single markdown file and return content chunks
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract frontmatter if present
        frontmatter, body = self._extract_frontmatter(content)

        # Convert markdown to HTML to extract sections
        html_content = markdown.markdown(body)
        soup = BeautifulSoup(html_content, 'html.parser')

        # Build the URL based on file path
        relative_path = file_path.relative_to(self.base_path)
        url = self._convert_path_to_url(relative_path)

        # Extract title from H1 or frontmatter
        title = frontmatter.get('title') if frontmatter else self._extract_title(soup)

        # Extract sections (H2, H3) to create content chunks
        chunks = []
        current_chunk = {
            'title': title,
            'url': url,
            'chapter': frontmatter.get('sidebar_label') or title,
            'section': '',
            'content': '',
            'file_path': str(file_path)
        }

        # Split content by headings to create chunks
        elements = soup.find_all(['h1', 'h2', 'h3', 'p', 'li', 'code', 'pre'])

        for element in elements:
            if element.name in ['h2', 'h3']:
                # If we have content in the current chunk, save it
                if current_chunk['content'].strip():
                    chunks.append(current_chunk.copy())

                # Start a new chunk with the heading
                current_chunk['section'] = element.get_text().strip()
                current_chunk['content'] = ''
            elif element.name in ['p', 'li', 'code', 'pre']:
                text = element.get_text().strip()
                if text:
                    current_chunk['content'] += text + '\n\n'

        # Add the last chunk if it has content
        if current_chunk['content'].strip():
            chunks.append(current_chunk)
        elif not chunks and current_chunk['title']:
            # If no sections were found, create one chunk with all content
            current_chunk['section'] = title
            current_chunk['content'] = body
            chunks.append(current_chunk)

        return chunks

    def _extract_frontmatter(self, content: str) -> tuple:
        """
        Extract YAML frontmatter from markdown content
        """
        if content.startswith('---'):
            try:
                end_frontmatter = content.find('---', 3)
                if end_frontmatter != -1:
                    frontmatter_str = content[3:end_frontmatter].strip()
                    frontmatter = yaml.safe_load(frontmatter_str)
                    body = content[end_frontmatter + 3:].strip()
                    return frontmatter, body
            except yaml.YAMLError:
                pass

        return {}, content

    def _extract_title(self, soup: BeautifulSoup) -> str:
        """
        Extract title from H1 tag in HTML
        """
        h1 = soup.find('h1')
        if h1:
            return h1.get_text().strip()
        return "Untitled"

    def _convert_path_to_url(self, relative_path: Path) -> str:
        """
        Convert file path to Docusaurus URL
        """
        # Remove .md extension and convert to URL format
        path_str = str(relative_path.with_suffix(''))

        # Replace backslashes with forward slashes
        path_str = path_str.replace('\\', '/')

        # If it's in a subdirectory, adjust the URL
        if path_str.startswith('docs/'):
            path_str = path_str[5:]  # Remove 'docs/'

        # Handle index files
        if path_str.endswith('/index') or path_str == 'index':
            path_str = path_str.replace('/index', '').replace('index', '')

        # Add leading slash if not present
        if not path_str.startswith('/'):
            path_str = '/' + path_str

        # If empty path, return root
        if not path_str or path_str == '/':
            return '/'

        return path_str