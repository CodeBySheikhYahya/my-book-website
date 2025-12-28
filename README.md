# My Book Website

A book website with an interactive chatbot powered by RAG (Retrieval Augmented Generation).

## Architecture

This project follows a **spec-driven architecture**. The API is defined in `openapi.yaml`, which serves as the single source of truth. See [SPEC-DRIVEN.md](./SPEC-DRIVEN.md) for details.

## Quick Start

### Backend

```bash
cd backend
uv venv
.venv\Scripts\Activate.ps1  # Windows
uv pip install -r requirements.txt
uvicorn app:app --reload
```

### Frontend

```bash
npm install
npm start
```

## API Specification

The API is defined in `openapi.yaml`. To validate that the backend matches the spec:

```bash
python backend/validate_spec.py
```

To regenerate the frontend API client from the spec:

```bash
npm run generate-api
```





