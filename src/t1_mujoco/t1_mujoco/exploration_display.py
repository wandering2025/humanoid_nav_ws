"""Read-only display coordinates from the explorer, never from scene metadata."""
import math


def start_markers(event):
    """Return immutable A/B odom coordinates, or None for unrelated/invalid events."""
    if not isinstance(event, dict) or event.get('event') != 'start':
        return None
    anchor, goal = event.get('anchor_odom'), event.get('goal_odom')
    if not isinstance(anchor, (list, tuple)) or not isinstance(goal, (list, tuple)):
        return None
    if len(anchor) != 3 or len(goal) != 2:
        return None
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (*anchor, *goal)):
        return None
    return tuple(float(v) for v in anchor), tuple(float(v) for v in goal)
