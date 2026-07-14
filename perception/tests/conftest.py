"""
Make the perception/ package importable from tests/ regardless of which
directory pytest is invoked from (matters once this runs in CI or from the
repo root rather than always from inside perception/).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
