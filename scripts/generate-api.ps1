# Generate TypeScript API client from OpenAPI spec (PowerShell)

Write-Host "🔧 Generating API client from OpenAPI spec..." -ForegroundColor Cyan

# Navigate to project root
Set-Location $PSScriptRoot\..

# Generate API client
npm run generate-api

Write-Host "✅ API client generated in src/api/" -ForegroundColor Green
Write-Host "📝 Remember to update your components to use the generated client" -ForegroundColor Yellow

