"""Post adapter, rear: clamps a rear chassis body post and carries a rear
deck mounting point (two per car, the right one turned 180 deg)."""
from hardware.parts.racecarneo import _adapter

PART = _adapter.part("rear")


def build():
    return _adapter.build("rear")
