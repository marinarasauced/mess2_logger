
from datetime import datetime
from os import listdir, path
import re


PATTERN = re.compile(r'^(\d{4})_')


def get_trial_dir_path(log_dir_path: str) -> str:
    """
    """
    log_dir_path = path.expanduser(log_dir_path)
    index_max = 0
    if path.exists(log_dir_path):
        for name in listdir(log_dir_path):
            match = PATTERN.match(name)
            if match:
                try:
                    index = int(match.group(1))
                    index_max = max(index_max, index)
                except ValueError:
                    continue
    index_next = index_max + 1
    timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    return path.join(log_dir_path, f"{index_next:04d}_{timestamp}")
