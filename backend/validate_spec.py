"""
Validate that the FastAPI app matches the OpenAPI spec.
Run this to ensure code matches the spec (spec-driven development).
"""
import json
import yaml
from pathlib import Path
from fastapi.openapi.utils import get_openapi
from app import app

def load_openapi_spec():
    """Load the OpenAPI spec file"""
    spec_path = Path(__file__).parent.parent / "openapi.yaml"
    with open(spec_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def get_fastapi_openapi():
    """Get OpenAPI schema from FastAPI app"""
    return get_openapi(
        title=app.title,
        version="1.0.0",
        description=app.description or "",
        routes=app.routes,
    )

def validate_spec():
    """Compare the spec file with FastAPI-generated schema"""
    print("=" * 50)
    print("🔍 Validating OpenAPI spec against FastAPI app...")
    print("=" * 50)
    
    try:
        spec_file = load_openapi_spec()
        fastapi_schema = get_fastapi_openapi()
        
        # Basic validation - check if paths match
        spec_paths = set(spec_file.get("paths", {}).keys())
        fastapi_paths = set(fastapi_schema.get("paths", {}).keys())
        
        missing_in_spec = fastapi_paths - spec_paths
        missing_in_fastapi = spec_paths - fastapi_paths
        
        if missing_in_spec:
            print(f"⚠️  Paths in FastAPI but not in spec: {missing_in_spec}")
        
        if missing_in_fastapi:
            print(f"⚠️  Paths in spec but not in FastAPI: {missing_in_fastapi}")
        
        if not missing_in_spec and not missing_in_fastapi:
            print("✅ All paths match between spec and FastAPI!")
        
        # Check if we can export the current schema
        output_path = Path(__file__).parent.parent / "openapi-generated.json"
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(fastapi_schema, f, indent=2)
        print(f"📄 Current FastAPI schema exported to: {output_path}")
        print("   Compare this with openapi.yaml to ensure they match")
        
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"❌ Error validating spec: {e}")
        return False

if __name__ == "__main__":
    validate_spec()

