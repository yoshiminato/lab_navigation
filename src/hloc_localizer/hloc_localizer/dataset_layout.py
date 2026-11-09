"""Paths shared by map generation and online localization.

``data_path`` always names a capture session, never a method directory.
Adding a local feature method requires one entry here and its backend.
"""

from pathlib import Path


METHOD_DIRECTORIES = {
    'sift': 'sift',
    'superpoint': 'superpoint',
    'superpoint_lightglue': 'superpoint',
}


def map_dir(data_path, local_method):
    try:
        name = METHOD_DIRECTORIES[local_method]
    except KeyError as exc:
        raise ValueError(f'Unknown local method: {local_method}') from exc
    return Path(data_path) / 'maps' / name


def model_dir(data_path, local_method):
    return map_dir(data_path, local_method) / 'sparse' / '0'


def reference_features(data_path, local_method):
    return map_dir(data_path, local_method) / 'reference_features.h5'


def retrieval_features(data_path):
    return Path(data_path) / 'retrieval' / 'netvlad.h5'
