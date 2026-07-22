from pathlib import Path


def deploy_root() -> Path:
    return Path(__file__).resolve().parents[1]


def repo_root() -> Path:
    return deploy_root()


def resolve_repo_path(path: str | Path) -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return candidate
    return repo_root() / candidate


def ensure_dir(path: str | Path) -> Path:
    resolved = resolve_repo_path(path)
    resolved.mkdir(parents=True, exist_ok=True)
    return resolved
