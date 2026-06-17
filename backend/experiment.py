import csv
import json
import os
from datetime import datetime
from typing import Dict


class ExperimentLogger:
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self.events_file = os.path.join(self.log_dir, "events.jsonl")
        self.metrics_file = os.path.join(self.log_dir, "metrics.csv")

        if not os.path.exists(self.metrics_file):
            with open(self.metrics_file, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                writer.writerow([
                    "timestamp",
                    "participant_id",
                    "operator_type",
                    "interface_variant",
                    "task_id",
                    "task_time_sec",
                    "error_count",
                    "mission_success",
                    "sus_score",
                    "nasa_tlx",
                ])

    def log_event(self, event: str, payload: Dict, operator_type: str = "unknown", interface_variant: str = "full"):
        row = {
            "timestamp": datetime.utcnow().isoformat(),
            "event": event,
            "operator_type": operator_type,
            "interface_variant": interface_variant,
            "payload": payload,
        }
        with open(self.events_file, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(row) + "\n")

    def log_metric(self, metric: Dict):
        with open(self.metrics_file, "a", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow([
                datetime.utcnow().isoformat(),
                metric.get("participant_id", ""),
                metric.get("operator_type", ""),
                metric.get("interface_variant", ""),
                metric.get("task_id", ""),
                metric.get("task_time_sec", ""),
                metric.get("error_count", ""),
                metric.get("mission_success", ""),
                metric.get("sus_score", ""),
                metric.get("nasa_tlx", ""),
            ])
