"""Simulated operator: one trigger request per centered target identity.

Imported only by simulation plants. It never arms or reloads a launcher and
never retries a failed firing cycle. The tracker still validates every
request through its normal fire gates.
"""
import math


class SimulationTrigger:
    def __init__(self, action='fire'):
        self.action = action
        self.attempted = set()

    def command(self, status, now):
        if not isinstance(status, dict):
            return None
        stamp = status.get('stamp')
        if not isinstance(stamp, (int, float)) or isinstance(stamp, bool):
            return None
        if not math.isfinite(stamp) or not 0 <= now - stamp <= .5:
            return None
        target = status.get('target_id')
        if not target or target in self.attempted or target in status.get('tagged_ids', []):
            return None
        if status.get('launcher') != 'ready' or status.get('fire_block') != '':
            return None
        if self.action == 'tag' and not status.get('locked'):
            return None
        self.attempted.add(target)
        return {'action': self.action, 'target_id': target}
