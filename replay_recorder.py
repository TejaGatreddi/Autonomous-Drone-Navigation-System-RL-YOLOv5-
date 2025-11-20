"""Optional helper to record state transitions during training for offline analysis.
Records to a CSV with basic columns (timestamp, obs, action, reward, done).
"""
import csv
import os
import time
import json

class ReplayRecorder:
    def __init__(self, out_path="data/replay.csv"):
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        self.out_path = out_path
        self._init_file()

    def _init_file(self):
        self.fh = open(self.out_path, "w", newline="")
        self.writer = csv.writer(self.fh)
        self.writer.writerow(["ts", "obs_json", "action_json", "reward", "done"])

    def record(self, obs, action, reward, done):
        self.writer.writerow([time.time(), json.dumps(obs.tolist() if hasattr(obs, "tolist") else obs), json.dumps(action.tolist() if hasattr(action, "tolist") else action), reward, done])
        self.fh.flush()

    def close(self):
        self.fh.close()
