#!/bin/bash
# Generate TypeScript API client from OpenAPI spec

echo "🔧 Generating API client from OpenAPI spec..."

# Navigate to project root
cd "$(dirname "$0")/.."

# Generate API client
npm run generate-api

echo "✅ API client generated in src/api/"
echo "📝 Remember to update your components to use the generated client"

