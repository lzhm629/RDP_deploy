from pathlib import Path
import sys

RDP_DEPLOY_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = RDP_DEPLOY_DIR.parent

for path in (RDP_DEPLOY_DIR, REPO_DIR / "reactive_diffusion_policy"):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)
