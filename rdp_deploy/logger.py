import sys
from pathlib import Path

from loguru import logger

from rdp_deploy.paths import resolve_repo_path


def setup_logger(level: str = "INFO", log_file: str | None = None):
    logger.remove()
    logger.add(sys.stderr, level=level)
    if log_file:
        path = resolve_repo_path(log_file)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        logger.add(path, level=level, rotation="20 MB", retention=10)
    return logger
