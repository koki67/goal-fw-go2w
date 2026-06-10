"""Pure helpers for the velocity bridge. No ROS imports — unit testable."""
from __future__ import annotations

import json
import math
from typing import Tuple


def clamp(value: float, cap: float) -> float:
    """Clamp a scalar to [-cap, +cap]. Invalid inputs force zero."""
    if not math.isfinite(value) or not math.isfinite(cap) or cap <= 0.0:
        return 0.0
    if value > cap:
        return cap
    if value < -cap:
        return -cap
    return value


def clamp_twist(
    vx: float,
    vy: float,
    wz: float,
    vx_max: float,
    vy_max: float,
    wz_max: float,
) -> Tuple[float, float, float]:
    """Clamp the three Sport-API velocity components to their per-axis caps."""
    values = (vx, vy, wz, vx_max, vy_max, wz_max)
    if not all(math.isfinite(value) for value in values):
        return 0.0, 0.0, 0.0
    return clamp(vx, vx_max), clamp(vy, vy_max), clamp(wz, wz_max)


def encode_move_parameter(vx: float, vy: float, wz: float, decimals: int = 4) -> str:
    """JSON-encode the Sport-API Move parameter (api_id 1008).

    Decimals cap helps keep the message body small and stable against tiny
    floating-point drift; the Sport API ignores precision below ~1e-3 anyway.
    """
    return json.dumps({
        "x": round(vx, decimals),
        "y": round(vy, decimals),
        "z": round(wz, decimals),
    }, allow_nan=False)
