"""
Make evaluation/ importable from tests/ regardless of which directory
pytest is invoked from -- same convention as perception/tests/conftest.py.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
