from __future__ import annotations

import os
from typing import Mapping, Optional

from scikit_build_core import build as _sbc


def _is_truthy(value) -> bool:  
    # config_settings values are sometimes lists
    if isinstance(value, list):
        value = value[-1] if value else ""

    return str(value).strip().upper() in {"ON", "1", "TRUE", "YES"}

def _jit_requested(config_settings: Optional[Mapping] = None) -> bool:
    if _is_truthy(os.environ.get("VAMP_BUILD_JIT", "")):
        return True

    cs = config_settings or {}
    if _is_truthy(cs.get("cmake.define.VAMP_BUILD_JIT", "")):
        return True

    cmake_define = cs.get("cmake.define", {})
    if isinstance(cmake_define, Mapping) and _is_truthy(cmake_define.get("VAMP_BUILD_JIT", "")):
        return True

    return False

def _with_cricket(base, config_settings):
    base = list(base or [])
    if _jit_requested(config_settings):
        base.append("cricket")
    return base


# PEP-517 Hooks
def get_requires_for_build_wheel(config_settings=None):
    return _with_cricket(_sbc.get_requires_for_build_wheel(config_settings), config_settings)

def get_requires_for_build_editable(config_settings=None):
    return _with_cricket(_sbc.get_requires_for_build_editable(config_settings), config_settings)

def get_requires_for_build_sdist(config_settings=None):
    return _sbc.get_requires_for_build_sdist(config_settings) or []


build_wheel = _sbc.build_wheel
build_editable = _sbc.build_editable
build_sdist = _sbc.build_sdist
prepare_metadata_for_build_wheel = _sbc.prepare_metadata_for_build_wheel
prepare_metadata_for_build_editable = _sbc.prepare_metadata_for_build_editable