# Spec-Driven Architecture Guide

This project now follows a **spec-driven architecture**, where the OpenAPI specification (`openapi.yaml`) is the single source of truth for the API.

## What is Spec-Driven Development?

- **Spec First**: The API specification (`openapi.yaml`) defines all endpoints, request/response formats, and types
- **Code Follows Spec**: Backend and frontend code should match the spec
- **Generated Clients**: Frontend API clients are generated from the spec
- **Validation**: Tools ensure code matches the spec

## Project Structure

```
├── openapi.yaml              # 📋 API Specification (SOURCE OF TRUTH)
├── backend/
│   ├── app.py                # FastAPI app (should match spec)
│   ├── validate_spec.py      # Validates code matches spec
│   └── ...
├── src/
│   ├── api/
│   │   └── client.ts         # Generated API client (from spec)
│   └── ...
└── scripts/
    └── generate-api.*        # Scripts to generate API client
```

## Workflow

### 1. Update the API Spec

When you need to change the API:

1. **Edit `openapi.yaml`** first (add/modify endpoints, types, etc.)
2. **Validate** that your FastAPI code matches: `python backend/validate_spec.py`
3. **Update FastAPI code** in `backend/app.py` to match the spec
4. **Regenerate frontend client**: `npm run generate-api`
5. **Update frontend** to use new types/methods

### 2. Validate Backend Matches Spec

```bash
cd backend
python validate_spec.py
```

This will:
- Compare your FastAPI app with `openapi.yaml`
- Show any mismatches
- Export current FastAPI schema to `openapi-generated.json` for comparison

### 3. Generate Frontend API Client

```bash
npm run generate-api
```

This generates TypeScript types and API client methods in `src/api/` from the OpenAPI spec.

**Note**: Currently using a manual client wrapper. After running `npm install`, you can regenerate the full client.

### 4. Using the API Client

The frontend now uses the generated API client:

```typescript
import { ApiClient } from '../api/client';

const apiClient = new ApiClient('http://localhost:8000');

// Use typed methods
const response = await apiClient.chat({
  question: "What is Physical AI?",
  selected_text: "optional context",
  session_id: "optional-session-id"
});
```

## Benefits

✅ **Type Safety**: Frontend gets TypeScript types from the spec  
✅ **Consistency**: Backend and frontend always match  
✅ **Documentation**: The spec serves as living documentation  
✅ **Validation**: Catch mismatches before deployment  
✅ **Code Generation**: Less manual work, fewer bugs  

## Next Steps

1. **Install dependencies**: `npm install` (to get code generation tools)
2. **Run validation**: `python backend/validate_spec.py`
3. **Generate client**: `npm run generate-api` (after installing)
4. **Keep spec updated**: Always update `openapi.yaml` first when changing the API

## Files to Know

- `openapi.yaml` - **Edit this first** when changing the API
- `backend/validate_spec.py` - Run this to check code matches spec
- `src/api/client.ts` - Generated API client (can be regenerated)
- `scripts/generate-api.*` - Scripts to regenerate client

