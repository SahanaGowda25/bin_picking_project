import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class EmergencyButtonNode(Node):
    def __init__(self):
        super().__init__('emergency_button_node')
        self.emergency_pressed = False

        self.publisher = self.create_publisher(Bool, 'emergency_state', 10)

        self.press_service = self.create_service(Trigger, 'press_emergency', self.press_callback)
        self.release_service = self.create_service(Trigger, 'release_emergency', self.release_callback)

        self.timer = self.create_timer(5.0, self.publish_emergency_state)
        self.get_logger().info("Emergency Button Node has started.")

    def publish_emergency_state(self):
        msg = Bool()
        msg.data = self.emergency_pressed
        self.publisher.publish(msg)
        self.get_logger().info(f"Emergency state: {'PRESSED' if self.emergency_pressed else 'RELEASED'}")

    def press_callback(self, request, response):
        self.emergency_pressed = True
        response.success = True
        response.message = "Emergency button pressed!"
        self.get_logger().info(response.message)
        return response

    def release_callback(self, request, response):
        self.emergency_pressed = False
        response.success = True
        response.message = "Emergency button released!"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
