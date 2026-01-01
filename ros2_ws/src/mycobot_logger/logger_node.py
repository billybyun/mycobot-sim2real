import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LoggerNode(Node):
    """
    Minimal logger stub:
      - subscribes to /mycobot/bridge_status
      - writes CSV rows into data/real/runs/<run_id>/telemetry.csv
    """

    def __init__(self):
        super().__init__("mycobot_logger")

        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
        out_dir = os.path.join(repo_root, "data", "real", "runs", run_id)
        os.makedirs(out_dir, exist_ok=True)
        self.csv_path = os.path.join(out_dir, "telemetry.csv")

        self.get_logger().info(f"Logging to: {self.csv_path}")

        self.f = open(self.csv_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "topic", "data"])
        self.f.flush()

        self.sub = self.create_subscription(String, "/mycobot/bridge_status", self.on_msg, 10)

    def on_msg(self, msg: String):
        t = self.get_clock().now().to_msg()
        ts = f"{t.sec}.{t.nanosec:09d}"
        self.w.writerow([ts, "/mycobot/bridge_status", msg.data])
        self.f.flush()

    def destroy_node(self):
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

