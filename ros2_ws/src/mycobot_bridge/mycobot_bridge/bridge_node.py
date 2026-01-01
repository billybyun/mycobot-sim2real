import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BridgeNode(Node):
    """
    Minimal, honest bridge stub:
      - subscribes to /mycobot/cmd (String)
      - republishes /mycobot/bridge_status (String heartbeat + last command)
    This proves ROS2 package structure & wiring without claiming full robot control yet.
    """

    def __init__(self):
        super().__init__("mycobot_bridge")
        self.last_cmd = "<none>"

        self.sub_cmd = self.create_subscription(String, "/mycobot/cmd", self.on_cmd, 10)
        self.pub_status = self.create_publisher(String, "/mycobot/bridge_status", 10)

        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info("mycobot_bridge started (stub). Listening on /mycobot/cmd")

    def on_cmd(self, msg: String):
        self.last_cmd = msg.data
        self.get_logger().info(f"Received cmd: {self.last_cmd}")

    def on_timer(self):
        msg = String()
        msg.data = f"bridge_ok last_cmd={self.last_cmd}"
        self.pub_status.publish(msg)


def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

