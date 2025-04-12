import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool



class DoorHandleNode(Node):
    def __init__(self):
        super().__init__('door_handle')
        self.door_closed = True  # Assume door starts closed

        self.publisher = self.create_publisher(Bool, 'door_state', 10)
        self.service = self.create_service(SetBool, 'toggle_door', self.toggle_door_callback)

        self.timer = self.create_timer(5.0, self.publish_door_state)
        self.get_logger().info("Door Handle Node has started.")

    def publish_door_state(self):
        msg = Bool()
        msg.data = self.door_closed
        self.publisher.publish(msg)
        self.get_logger().info(f'Published door state: {"closed" if self.door_closed else "open"}')

    def toggle_door_callback(self, request, response):
        self.door_closed = request.data
        response.success = True
        response.message = f'Door is now {"closed" if self.door_closed else "open"}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DoorHandleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
