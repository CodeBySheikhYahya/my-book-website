"""
Services for book chatbot - Qdrant and OpenAI (Gemini) integration
"""
import os
from typing import List, Optional
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables first
load_dotenv()

# Initialize clients lazily to avoid import-time errors
openai_client = None
qdrant_client = None

def get_openai_client():
    """Get or create OpenAI client (pointing to Groq)"""
    global openai_client
    if openai_client is None:
        try:
            import httpx
            # Create custom httpx client
            http_client = httpx.Client(timeout=60.0)
            # Use Groq for chat (OpenAI-compatible)
            openai_client = OpenAI(
                api_key=os.getenv("GROQ_API_KEY") or "",
                base_url="https://api.groq.com/openai/v1",
                http_client=http_client
            )
        except Exception as e:
            print(f"Warning: Could not create OpenAI client: {e}")
            openai_client = None
    return openai_client

def get_qdrant_client():
    """Get or create Qdrant client"""
    global qdrant_client
    if qdrant_client is None:
        qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
        )
    return qdrant_client

COLLECTION_NAME = "book_chunks"
EMBEDDING_MODEL = "text-embedding-004"  # Gemini embedding model


def get_embedding(text: str) -> List[float]:
    """Get embedding for text using Gemini API directly"""
    try:
        # Use Google's native API for embeddings (more reliable)
        import requests
        
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            print("GEMINI_API_KEY not found")
            return []
            
        url = f"https://generativelanguage.googleapis.com/v1beta/models/text-embedding-004:embedContent?key={api_key}"
        
        payload = {
            "model": "models/text-embedding-004",
            "content": {"parts": [{"text": text}]},
            "taskType": "RETRIEVAL_DOCUMENT"
        }
        
        response = requests.post(url, json=payload, timeout=30)
        if response.status_code == 200:
            data = response.json()
            return data.get("embedding", {}).get("values", [])
        else:
            print(f"Embedding API error: {response.status_code} - {response.text[:200]}")
            return []
    except Exception as e:
        print(f"Error getting embedding: {e}")
        return []


def create_collection_if_not_exists():
    """Create Qdrant collection if it doesn't exist"""
    try:
        client = get_qdrant_client()
        collections = client.get_collections()
        collection_names = [col.name for col in collections.collections]
        
        if COLLECTION_NAME not in collection_names:
            client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=768,  # Gemini text-embedding-004 size
                    distance=Distance.COSINE
                )
            )
            print(f"Created collection: {COLLECTION_NAME}")
        else:
            print(f"Collection {COLLECTION_NAME} already exists")
    except Exception as e:
        print(f"Error creating collection: {e}")


def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """Split text into chunks with overlap"""
    words = text.split()
    chunks = []
    
    for i in range(0, len(words), chunk_size - overlap):
        chunk = " ".join(words[i:i + chunk_size])
        if chunk.strip():
            chunks.append(chunk)
    
    return chunks


def process_markdown_file(file_path: Path, docs_base_path: Path) -> List[dict]:
    """Process a markdown file and return chunks with metadata"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract title from frontmatter or first heading
        title = file_path.stem
        if content.startswith('---'):
            # Try to extract title from frontmatter
            parts = content.split('---', 2)
            if len(parts) > 2:
                frontmatter = parts[1]
                if 'title:' in frontmatter:
                    title = frontmatter.split('title:')[1].split('\n')[0].strip()
        
        # Split into chunks
        chunks = chunk_text(content)
        
        # Create chunk objects with metadata
        chunk_objects = []
        for i, chunk in enumerate(chunks):
            # Get relative path from docs folder
            try:
                relative_path = str(file_path.relative_to(docs_base_path))
            except:
                relative_path = str(file_path.name)
            
            chunk_objects.append({
                'text': chunk,
                'source': relative_path,
                'title': title,
                'chunk_index': i
            })
        
        return chunk_objects
    except Exception as e:
        print(f"Error processing {file_path}: {e}")
        return []


def index_book_content(docs_folder: str = None):
    """Process all markdown files and index them in Qdrant"""
    if docs_folder is None:
        # Default to ../docs relative to backend folder
        backend_dir = Path(__file__).parent
        docs_path = backend_dir.parent / "docs"
    else:
        docs_path = Path(docs_folder)
    
    if not docs_path.exists():
        print(f"Docs folder not found: {docs_path}")
        return
    
    # Create collection
    create_collection_if_not_exists()
    
    # Find all markdown files
    markdown_files = list(docs_path.rglob("*.md"))
    print(f"Found {len(markdown_files)} markdown files")
    
    all_chunks = []
    for md_file in markdown_files:
        chunks = process_markdown_file(md_file, docs_path)
        all_chunks.extend(chunks)
        print(f"Processed {md_file.name}: {len(chunks)} chunks")
    
    print(f"Total chunks: {len(all_chunks)}")
    
    # Get embeddings and upload to Qdrant
    points = []
    for idx, chunk in enumerate(all_chunks):
        embedding = get_embedding(chunk['text'])
        if embedding:
            points.append(
                PointStruct(
                    id=idx,
                    vector=embedding,
                    payload={
                        'text': chunk['text'],
                        'source': chunk['source'],
                        'title': chunk['title'],
                        'chunk_index': chunk['chunk_index']
                    }
                )
            )
        
        if (idx + 1) % 10 == 0:
            print(f"Processed {idx + 1}/{len(all_chunks)} chunks")
    
    # Upload to Qdrant
    if points:
        client = get_qdrant_client()
        client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print(f"Indexed {len(points)} chunks in Qdrant")
    
    return len(points)


def search_relevant_chunks(query: str, limit: int = 5) -> List[dict]:
    """Search for relevant chunks in Qdrant"""
    try:
        # Get query embedding
        query_embedding = get_embedding(query)
        if not query_embedding:
            return []
        
        # Search in Qdrant
        client = get_qdrant_client()
        results = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=limit
        )
        
        # Format results
        chunks = []
        for result in results:
            chunks.append({
                'text': result.payload.get('text', ''),
                'source': result.payload.get('source', ''),
                'title': result.payload.get('title', ''),
                'score': result.score
            })
        
        return chunks
    except Exception as e:
        print(f"Error searching chunks: {e}")
        return []


def get_chat_response(question: str, selected_text: Optional[str] = None) -> str:
    """Get chat response using Groq with RAG"""
    try:
        # Build context from relevant chunks
        relevant_chunks = search_relevant_chunks(question, limit=5)
        
        # Build context
        context = "Book Content:\n\n"
        for chunk in relevant_chunks:
            context += f"From {chunk['title']}:\n{chunk['text']}\n\n"
        
        # Add selected text if provided
        if selected_text:
            context += f"\nUser selected text:\n{selected_text}\n\n"
        
        # Build messages
        messages = [
            {
                "role": "system",
                "content": "You are a helpful assistant that answers questions about the book 'Physical AI & Humanoid Robotics'. Use the provided book content to answer questions accurately. If the answer is not in the book content, say so."
            },
            {
                "role": "user",
                "content": f"{context}\n\nQuestion: {question}\n\nAnswer based on the book content above:"
            }
        ]
        
        # Get response from Groq (OpenAI-compatible)
        try:
            client = get_openai_client()
            if client:
                response = client.chat.completions.create(
                    model="llama-3.3-70b-versatile",  # Latest free Groq model
                    messages=messages,
                    temperature=0.7
                )
                return response.choices[0].message.content
            else:
                # Fallback: use Groq API directly
                import requests
                api_key = os.getenv("GROQ_API_KEY")
                if not api_key:
                    return "Error: GROQ_API_KEY not found"
                
                url = "https://api.groq.com/openai/v1/chat/completions"
                headers = {
                    "Authorization": f"Bearer {api_key}",
                    "Content-Type": "application/json"
                }
                
                payload = {
                    "model": "llama-3.3-70b-versatile",  # Latest free Groq model
                    "messages": messages,
                    "temperature": 0.7
                }
                
                response = requests.post(url, json=payload, headers=headers, timeout=30)
                if response.status_code == 200:
                    data = response.json()
                    return data["choices"][0]["message"]["content"]
                else:
                    return f"Error: {response.status_code} - {response.text[:200]}"
        except Exception as e:
            return f"Error: {str(e)}"
    except Exception as e:
        print(f"Error getting chat response: {e}")
        return f"Error: {str(e)}"

