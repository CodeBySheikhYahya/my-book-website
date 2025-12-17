"""
Database connection and setup for Neon Postgres
"""
import os
import json
import psycopg
from psycopg.types.json import Jsonb
from contextlib import contextmanager
from dotenv import load_dotenv

load_dotenv()

# Get database URL from environment
DATABASE_URL = os.getenv("DATABASE_URL")

# Debug: Check if DATABASE_URL is loaded and clean it
if DATABASE_URL:
    # Remove any quotes that might be around the value
    DATABASE_URL = DATABASE_URL.strip().strip('"').strip("'")
    # Check if it starts with "DATABASE_URL=" (wrong format)
    if DATABASE_URL.startswith("DATABASE_URL="):
        print("❌ ERROR: DATABASE_URL value contains 'DATABASE_URL=' prefix!")
        print(f"   Current value: {DATABASE_URL[:80]}...")
        print("   Fix: Remove 'DATABASE_URL=' from the value in .env file")
        DATABASE_URL = DATABASE_URL.replace("DATABASE_URL=", "", 1)
        print(f"   Fixed to: {DATABASE_URL[:80]}...")
    
    print(f"✅ DATABASE_URL loaded: {DATABASE_URL[:50]}...")
    print(f"   Length: {len(DATABASE_URL)} chars")
    print(f"   Starts with: {DATABASE_URL[:20]}...")
else:
    print("❌ DATABASE_URL NOT FOUND in environment variables!")
    print("   Make sure DATABASE_URL is in backend/.env file")

def get_db_connection():
    """Get a database connection"""
    if not DATABASE_URL:
        print("❌ DATABASE_URL is None or empty")
        return None
    try:
        # Debug: Show what we're connecting with
        print(f"🔌 Attempting connection...")
        print(f"   Host: {DATABASE_URL.split('@')[1].split('/')[0] if '@' in DATABASE_URL else 'unknown'}")
        print(f"   Connection string (first 80 chars): {DATABASE_URL[:80]}...")
        
        # Check for common issues
        if "DATABASE_URL=" in DATABASE_URL:
            print("❌ ERROR: Connection string contains 'DATABASE_URL=' - this is wrong!")
            print("   The .env file likely has: DATABASE_URL=DATABASE_URL=...")
            return None
        
        conn = psycopg.connect(DATABASE_URL)
        print(f"   ✅ Connection successful!")
        return conn
    except Exception as e:
        print(f"❌ Error connecting to database: {e}")
        print(f"   Full connection string length: {len(DATABASE_URL)}")
        print(f"   First 100 chars: {DATABASE_URL[:100]}")
        import traceback
        traceback.print_exc()
        return None

@contextmanager
def get_db():
    """Context manager for database connections"""
    conn = get_db_connection()
    if conn is None:
        yield None
        return
    try:
        yield conn
        conn.commit()
    except Exception as e:
        conn.rollback()
        print(f"Database error: {e}")
        raise
    finally:
        conn.close()

def init_database():
    """Initialize database tables"""
    if not DATABASE_URL:
        print("⚠️ DATABASE_URL not set, skipping database initialization")
        return False
    
    try:
        print("🔧 Initializing database...")
        db_host = DATABASE_URL.split('@')[1].split('/')[0] if '@' in DATABASE_URL else 'database'
        print(f"   Connecting to: {db_host}")
        
        with get_db() as conn:
            if conn is None:
                print("❌ Database connection failed - check DATABASE_URL")
                return False
            
            with conn.cursor() as cur:
                # Create chat_history table
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS chat_history (
                        id SERIAL PRIMARY KEY,
                        session_id VARCHAR(255),
                        question TEXT NOT NULL,
                        answer TEXT NOT NULL,
                        selected_text TEXT,
                        sources JSONB,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                    )
                """)
                
                # Create index for faster queries
                cur.execute("""
                    CREATE INDEX IF NOT EXISTS idx_session_id 
                    ON chat_history(session_id)
                """)
                
                # Create index for timestamp
                cur.execute("""
                    CREATE INDEX IF NOT EXISTS idx_created_at 
                    ON chat_history(created_at)
                """)
                
            print("✅ Database initialized successfully")
            print("   ✓ Table: chat_history")
            print("   ✓ Indexes: idx_session_id, idx_created_at")
            return True
    except Exception as e:
        print(f"❌ Error initializing database: {e}")
        import traceback
        traceback.print_exc()
        return False

def save_chat_history(session_id: str, question: str, answer: str, 
                     selected_text: str = None, sources: list = None):
    """Save chat history to database"""
    if not DATABASE_URL:
        print("⚠️ DATABASE_URL not set - skipping database save")
        return False
    
    try:
        print(f"💾 Attempting to save chat history for session: {session_id[:8]}...")
        with get_db() as conn:
            if conn is None:
                print("❌ Database connection is None - check DATABASE_URL")
                return False
            
            # Convert sources list to JSONB
            sources_json = Jsonb(sources) if sources else None
            
            with conn.cursor() as cur:
                cur.execute("""
                    INSERT INTO chat_history 
                    (session_id, question, answer, selected_text, sources)
                    VALUES (%s, %s, %s, %s, %s)
                """, (session_id, question, answer, selected_text, sources_json))
            
            print(f"✅ Successfully saved chat history for session: {session_id[:8]}...")
            return True
    except Exception as e:
        print(f"❌ Error saving chat history: {e}")
        print(f"   Session ID: {session_id}")
        print(f"   Question: {question[:50]}...")
        import traceback
        traceback.print_exc()
        return False

def get_chat_history(session_id: str, limit: int = 10):
    """Get chat history for a session"""
    if not DATABASE_URL:
        return []
    
    try:
        with get_db() as conn:
            if conn is None:
                return []
            
            with conn.cursor() as cur:
                cur.execute("""
                    SELECT question, answer, selected_text, sources, created_at
                    FROM chat_history
                    WHERE session_id = %s
                    ORDER BY created_at DESC
                    LIMIT %s
                """, (session_id, limit))
                
                rows = cur.fetchall()
                return [
                    {
                        "question": row[0],
                        "answer": row[1],
                        "selected_text": row[2],
                        "sources": row[3],
                        "created_at": row[4].isoformat() if row[4] else None
                    }
                    for row in rows
                ]
    except Exception as e:
        print(f"Error getting chat history: {e}")
        return []

