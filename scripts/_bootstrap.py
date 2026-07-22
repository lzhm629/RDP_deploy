from pathlib import Path
import sys

RDP_DEPLOY_DIR = Path(__file__).resolve().parents[1]

for path in (RDP_DEPLOY_DIR,):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)
