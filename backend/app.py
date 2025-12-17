from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
import uuid
from dotenv import load_dotenv
from services import get_chat_response, search_relevant_chunks, index_book_content
from database import init_database, save_chat_history, get_chat_history

# Load environment variables
load_dotenv()

app = FastAPI(title="Book Chatbot API")

# Initialize database on startup
@app.on_event("startup")
async def startup_event():
    print("=" * 50)
    print("🚀 Starting Book Chatbot API...")
    print("=" * 50)
    print(f"📋 Checking DATABASE_URL...")
    db_url = os.getenv("DATABASE_URL")
    if db_url:
        print(f"   ✅ DATABASE_URL found: {db_url[:40]}...")
    else:
        print(f"   ❌ DATABASE_URL NOT SET!")
    print("=" * 50)
    init_database()
    print("=" * 50)

# Enable CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class ChatRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: Optional[List[dict]] = None
    session_id: Optional[str] = None

@app.get("/")
def read_root():
    return {"message": "Book Chatbot API is running"}

@app.get("/health")
def health_check():
    return {"status": "ok", "service": "book-chatbot-api"}

@app.get("/test-db")
def test_database():
    """Test database connection"""
    from database import DATABASE_URL, get_db_connection
    result = {
        "database_url_set": bool(DATABASE_URL),
        "database_url_preview": DATABASE_URL[:50] + "..." if DATABASE_URL else None,
        "connection_test": None,
        "error": None
    }
    
    if not DATABASE_URL:
        result["error"] = "DATABASE_URL not set in environment"
        return result
    
    try:
        conn = get_db_connection()
        if conn:
            result["connection_test"] = "✅ Successfully connected!"
            conn.close()
        else:
            result["connection_test"] = "❌ Connection failed"
    except Exception as e:
        result["connection_test"] = "❌ Error"
        result["error"] = str(e)
    
    return result

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Chat endpoint - answers questions about the book
    Can use selected text for context
    Stores chat history in Neon Postgres
    """
    try:
        print("\n" + "=" * 50)
        print("💬 New chat request received")
        print(f"   Question: {request.question[:50]}...")
        
        # Generate or use session ID
        session_id = request.session_id or str(uuid.uuid4())
        print(f"   Session ID: {session_id[:8]}...")
        
        # Get answer from chatbot
        print("   Getting answer from chatbot...")
        answer = get_chat_response(request.question, request.selected_text)
        print(f"   ✅ Got answer ({len(answer)} chars)")
        
        # Get relevant sources
        relevant_chunks = search_relevant_chunks(request.question, limit=3)
        sources = [
            {
                "title": chunk["title"],
                "source": chunk["source"],
                "text": chunk["text"][:200] + "..." if len(chunk["text"]) > 200 else chunk["text"]
            }
            for chunk in relevant_chunks
        ]
        
        # Save to Neon Postgres
        print(f"📝 Saving chat to database...")
        saved = save_chat_history(
            session_id=session_id,
            question=request.question,
            answer=answer,
            selected_text=request.selected_text,
            sources=sources
        )
        if saved:
            print(f"✅ Chat saved successfully")
        else:
            print(f"⚠️ Chat not saved (check logs above)")
        
        return {
            "answer": answer,
            "sources": sources,
            "session_id": session_id
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error: {str(e)}")

@app.post("/index")
async def index_books():
    """
    Index all book content into Qdrant
    Run this once to process all markdown files
    """
    try:
        count = index_book_content()
        return {"message": f"Indexed {count} chunks successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error: {str(e)}")

@app.get("/chat/history/{session_id}")
async def get_history(session_id: str, limit: int = 10):
    """
    Get chat history for a session
    """
    try:
        history = get_chat_history(session_id, limit)
        return {"session_id": session_id, "history": history}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error: {str(e)}")

