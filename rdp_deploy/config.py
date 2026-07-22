from pathlib import Path
from typing import Any

from omegaconf import DictConfig, OmegaConf

from rdp_deploy.paths import resolve_repo_path


def load_config(config_path: str | Path) -> DictConfig:
    path = resolve_repo_path(config_path)
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")
    cfg = OmegaConf.load(path)
    OmegaConf.resolve(cfg)
    return cfg


def cfg_get(cfg: DictConfig, dotted_key: str, default: Any = None) -> Any:
    cur: Any = cfg
    for part in dotted_key.split("."):
        if cur is None or part not in cur:
            return default
        cur = cur[part]
    return cur


def to_plain_container(cfg: Any) -> Any:
    return OmegaConf.to_container(cfg, resolve=True)


def resolve_config_paths(cfg: DictConfig) -> DictConfig:
    cfg = OmegaConf.create(to_plain_container(cfg))
    if cfg_get(cfg, "project.output_dir") is not None:
        cfg.project.output_dir = str(resolve_repo_path(cfg.project.output_dir))
    if cfg_get(cfg, "transforms.calibration_path") is not None:
        cfg.transforms.calibration_path = str(resolve_repo_path(cfg.transforms.calibration_path))
    pca_cfg = cfg_get(cfg, "data_processing.pca_param_dict")
    if pca_cfg is not None:
        for sensor_name in pca_cfg.keys():
            sensor_cfg = pca_cfg[sensor_name]
            for key in ("transformation_matrix_path", "mean_matrix_path"):
                if key in sensor_cfg and sensor_cfg[key] is not None:
                    sensor_cfg[key] = str(resolve_repo_path(sensor_cfg[key]))
    return cfg
