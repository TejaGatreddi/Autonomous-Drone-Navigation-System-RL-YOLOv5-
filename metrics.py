"""Small utilities to log simple training metrics and save CSV.
"""
import csv
import os

def save_metrics(path, metrics):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    keys = sorted(metrics.keys())
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(keys)
        writer.writerow([metrics[k] for k in keys])
