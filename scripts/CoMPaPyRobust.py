from pathlib import Path
from typing import Optional

from compapy.scripts.CoMPaPy import CoMPaPy


class CoMPaPyRobust(CoMPaPy):
    def __init__(self, log_file: Optional[Path] = None, save_planning_res: bool = False):
        super(CoMPaPyRobust, self).__init__(log_file=log_file, save_planning_res=save_planning_res)

    def move_with_fallback(self):
        pass
