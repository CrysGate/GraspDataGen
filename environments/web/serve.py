"""Launch the viewer from the isolated USD/NiceGUI environment."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from graspdatagen.web import main  # noqa: E402

if __name__ == "__main__":
    main()
