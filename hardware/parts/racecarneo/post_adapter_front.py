"""Post adapter, front: clamps a front chassis body post and carries a
front deck mounting point (two per car, the right one turned 180 deg)."""
from hardware.parts.racecarneo import _adapter

PART = _adapter.part("front")


def build():
    return _adapter.build("front")
