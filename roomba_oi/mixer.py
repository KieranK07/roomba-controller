"""Turn held keys into left/right wheel velocities."""

ARC_GAIN = 0.6      # how hard A/D bends the path while already moving


def mix(throttle: float, steer: float, spin: float,
        speed: float, turn_speed: float, max_mms: int = 500) -> tuple[int, int]:
    """
    throttle : -1..1  (W / S)      forward / back
    steer    : -1..1  (A / D)      arc while moving, tank-turn while stopped
    spin     : -1..1  (< / >)      always a pure turn in place
    Returns (left_mms, right_mms). Positive rotation = clockwise / to the right.
    """
    v = throttle * speed
    w = spin * turn_speed

    if throttle:
        w += steer * abs(v) * ARC_GAIN      # arc scales with travel speed
    else:
        w += steer * turn_speed             # stationary -> tank turn

    left, right = v + w, v - w

    # Scale rather than clip, so the turn ratio survives saturation.
    peak = max(abs(left), abs(right))
    if peak > max_mms:
        k = max_mms / peak
        left, right = left * k, right * k

    return int(round(left)), int(round(right))


def from_keys(keys: set[str], speed: float, turn_speed: float) -> tuple[int, int]:
    """WASD only. A/D arc while moving and tank-turn while stopped, which
    covers heading changes without a separate spin axis."""
    throttle = (("w" in keys) - ("s" in keys))
    steer    = (("d" in keys) - ("a" in keys))
    return mix(throttle, steer, 0, speed, turn_speed)
