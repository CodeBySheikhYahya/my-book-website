# Book Chatbot Backend

FastAPI backend for the book chatbot using Gemini (via OpenAI SDK) and Qdrant.

## Setup

1. Create virtual environment:
```bash
uv venv
```

2. Activate virtual environment:
```bash
# Windows PowerShell:
.venv\Scripts\Activate.ps1

# Windows CMD:
.venv\Scripts\activate.bat
```

3. Install dependencies:
```bash
uv pip install -r requirements.txt
```

4. Create `.env` file with your API keys:
```
# AI API Keys
GEMINI_API_KEY=your-gemini-api-key
GROQ_API_KEY=your-groq-api-key

# Qdrant Cloud
QDRANT_URL=https://your-cluster-url.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Serverless Postgres (optional but recommended)
DATABASE_URL=postgresql://user:password@host.neon.tech/database?sslmode=require
```

**Getting Neon Database URL:**
1. Go to https://neon.tech and sign up (free tier available)
2. Create a new project
3. Copy the connection string from the dashboard
4. Add it to `.env` as `DATABASE_URL`

5. Index book content (run once):
```bash
# Start server first
uvicorn app:app --reload

# Then in another terminal, call the index endpoint:
curl -X POST http://localhost:8000/index
```

6. Run the server:
```bash
uvicorn app:app --reload
```

7. Server will run at: http://localhost:8000

## API Endpoints

- `GET /` - API status
- `GET /health` - Health check
- `POST /chat` - Chat with the book
  ```json
  {
    "question": "What is Physical AI?",
    "selected_text": "optional selected text",
    "session_id": "optional session ID"
  }
  ```
  Returns:
  ```json
  {
    "answer": "Answer text",
    "sources": [...],
    "session_id": "session-uuid"
  }
  ```
- `POST /index` - Index all book content (run once)
- `GET /chat/history/{session_id}` - Get chat history for a session

## How It Works

1. **Indexing**: Reads all markdown files from `docs/` folder, splits into chunks, creates embeddings using Gemini, stores in Qdrant
2. **Chat**: When user asks question, searches Qdrant for relevant chunks, uses Groq (via OpenAI SDK) to generate answer based on book content
3. **Selected Text**: If user selects text, it's added as additional context
4. **Neon Postgres**: Stores chat history (questions, answers, sources) for each session. Database is initialized automatically on server startup.
