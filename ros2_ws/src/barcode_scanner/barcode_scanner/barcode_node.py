import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
import random
import time


class BarcodeScannerNode(Node):
    def __init__(self):
        super().__init__('barcode_scanner')
        self.barcode = self.generate_barcode()

        self.publisher = self.create_publisher(
            String,
            'barcode',
            QoSProfile(depth=10)
        )

        self.service = self.create_service(
            Trigger,
            'get_last_barcode',
            self.handle_get_barcode
        )

        self.timer = self.create_timer(5.0, self.publish_barcode)
        self.get_logger().info('Barcode scanner node started.')

    def generate_barcode(self):
        return str(random.randint(10000, 99999))

    def publish_barcode(self):
        self.barcode = self.generate_barcode()
        msg = String()
        msg.data = self.barcode
        self.publisher.publish(msg)
        self.get_logger().info(f'Published barcode: {msg.data}')

    def handle_get_barcode(self, request, response):
        response.success = True
        response.message = self.barcode
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BarcodeScannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
