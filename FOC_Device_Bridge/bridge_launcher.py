"""PyInstaller-safe entry point for the FOC Device Bridge package."""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from FOC_Device_Bridge.bridge_app import main


if __name__ == "__main__":
    raise SystemExit(main())
