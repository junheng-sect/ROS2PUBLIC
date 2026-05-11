#!/usr/bin/env python3

from functools import lru_cache
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


PROFILE_OVERRIDE_SENTINEL = '__profile__'
DEFAULT_CAMERA_PROFILE = 'old_cam'
CAMERA_PROFILE_CONFIG_RELATIVE_PATH = Path('config') / 'camera_profiles.yaml'
REQUIRED_CAMERA_PROFILE_KEYS = (
    'camera_name',
    'camera_info_url',
    'video_device',
    'image_width',
    'image_height',
    'pixel_format',
    'framerate',
    'camera_fx',
    'camera_fy',
    'camera_cx',
    'camera_cy',
    'dist_k1',
    'dist_k2',
    'dist_p1',
    'dist_p2',
    'dist_k3',
)


def _iter_candidate_config_paths():
    """按“安装空间优先、源码目录兜底”的顺序查找 profile 配置文件。"""
    try:
        share_dir = Path(get_package_share_directory('tvec'))
        yield share_dir / CAMERA_PROFILE_CONFIG_RELATIVE_PATH
    except PackageNotFoundError:
        pass

    # 当前文件位于 `tvec/tvec/`，向上一级回到包根目录。
    yield Path(__file__).resolve().parents[1] / CAMERA_PROFILE_CONFIG_RELATIVE_PATH


@lru_cache(maxsize=1)
def load_camera_profiles():
    """读取并校验相机 profile 配置。"""
    config_path = None
    for candidate in _iter_candidate_config_paths():
        if candidate.is_file():
            config_path = candidate
            break

    if config_path is None:
        raise FileNotFoundError(
            f'未找到相机 profile 配置文件：{CAMERA_PROFILE_CONFIG_RELATIVE_PATH}'
        )

    with config_path.open('r', encoding='utf-8') as file_obj:
        config = yaml.safe_load(file_obj) or {}

    profiles = config.get('profiles')
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError(f'相机 profile 配置为空：{config_path}')

    for profile_name, profile in profiles.items():
        if not isinstance(profile, dict):
            raise TypeError(f'camera_profile={profile_name} 的配置格式无效')
        missing_keys = [
            key for key in REQUIRED_CAMERA_PROFILE_KEYS if key not in profile
        ]
        if missing_keys:
            missing_text = ', '.join(missing_keys)
            raise ValueError(
                f'camera_profile={profile_name} 缺少字段：{missing_text}'
            )

    default_profile = str(
        config.get('default_profile', DEFAULT_CAMERA_PROFILE)
    ).strip()
    if default_profile not in profiles:
        raise ValueError(
            f'default_profile={default_profile} 未在 {config_path} 中定义'
        )

    return {
        'default_profile': default_profile,
        'profiles': profiles,
        'config_path': str(config_path),
    }


def get_default_camera_profile_name():
    """返回默认相机 profile 名称。"""
    return load_camera_profiles()['default_profile']


def list_camera_profiles():
    """返回已定义的相机 profile 名称列表。"""
    return sorted(load_camera_profiles()['profiles'].keys())


def get_camera_profile(profile_name):
    """按名称获取一份相机 profile 配置副本。"""
    profiles = load_camera_profiles()['profiles']
    if profile_name not in profiles:
        available_profiles = ', '.join(sorted(profiles.keys()))
        raise KeyError(
            f'未知 camera_profile={profile_name}，可选值：{available_profiles}'
        )
    return dict(profiles[profile_name])
