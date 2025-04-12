import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8


class StackLightNode(Node):
    def __init__(self):
        super().__init__('stack_light_node')

        self.door_closed = True
        self.emergency_pressed = False

        self.stack_pub = self.create_publisher(Int8, 'stack_light_state', 10)

        self.create_subscription(Bool, 'door_state', self.door_callback, 10)
        self.create_subscription(Bool, 'emergency_state', self.emergency_callback, 10)

        self.timer = self.create_timer(5.0, self.evaluate_state)
        self.get_logger().info("Stack Light Node has started.")

    def door_callback(self, msg):
        self.door_closed = msg.data
        self.get_logger().info(f"Door state received: {'closed' if self.door_closed else 'open'}")

    def emergency_callback(self, msg):
        self.emergency_pressed = msg.data
        self.get_logger().info(f"Emergency state received: {'PRESSED' if self.emergency_pressed else 'RELEASED'}")

    def evaluate_state(self):
        status = Int8()

        if self.emergency_pressed:
            status.data = -1  # Emergency
        elif not self.door_closed:
            status.data = 1   # Paused
        else:
            status.data = 0   # Operational

        self.stack_pub.publish(status)
        self.get_logger().info(f"Published stack light status: {status.data}")


def main(args=None):
    rclpy.init(args=args)
    node = StackLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
