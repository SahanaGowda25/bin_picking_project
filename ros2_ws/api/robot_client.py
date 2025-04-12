from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String
import threading

app = Flask(__name__)
rclpy.init()

class RobotClient(Node):
    def __init__(self):
        super().__init__('robot_client_node')

        self.door_closed = True
        self.emergency_pressed = False
        self.barcode = "UNKNOWN"

        self.create_subscription(Bool, 'door_state', self.door_callback, 10)
        self.create_subscription(Bool, 'emergency_state', self.emergency_callback, 10)
        self.create_subscription(String, 'barcode', self.barcode_callback, 10)

        self.get_logger().info("Robot client subscriptions initialized.")

    def door_callback(self, msg):
        self.door_closed = msg.data

    def emergency_callback(self, msg):
        self.emergency_pressed = msg.data

    def barcode_callback(self, msg):
        self.barcode = msg.data

robot_node = RobotClient()

@app.route('/confirmPick', methods=['POST'])
def confirm_pick():
    pick_id = request.get_json().get('pickId', 0)

    # Decision logic
    if robot_node.emergency_pressed:
        return jsonify({
            "pickId": pick_id,
            "pickSuccessful": False,
            "errorMessage": "Emergency stop is active",
            "itemBarcode": None
        })

    if not robot_node.door_closed:
        return jsonify({
            "pickId": pick_id,
            "pickSuccessful": False,
            "errorMessage": "Door is open",
            "itemBarcode": None
        })

    return jsonify({
        "pickId": pick_id,
        "pickSuccessful": True,
        "errorMessage": None,
        "itemBarcode": robot_node.barcode
    })

def ros_spin():
    rclpy.spin(robot_node)

def flask_run():
    app.run(host='0.0.0.0', port=8081)

# Start ROS 2 and Flask in parallel threads
threading.Thread(target=ros_spin, daemon=True).start()
flask_run()
