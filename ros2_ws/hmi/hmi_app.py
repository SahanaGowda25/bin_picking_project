from flask import Flask, render_template, jsonify, request
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import SetBool, Trigger
import threading

app = Flask(__name__)
rclpy.init()

class HMINode(Node):
    def __init__(self):
        super().__init__('hmi_node')

        self.door_closed = True
        self.emergency_pressed = False
        self.stack_status = 0
        self.barcode = "-----"

        # Subscriptions
        self.create_subscription(Bool, 'door_state', self.door_callback, 10)
        self.create_subscription(Bool, 'emergency_state', self.emergency_callback, 10)
        self.create_subscription(Int8, 'stack_light_state', self.stack_callback, 10)
        self.create_subscription(String, 'barcode', self.barcode_callback, 10)

        # Service clients
        self.door_toggle_client = self.create_client(SetBool, '/toggle_door')
        self.emergency_press_client = self.create_client(Trigger, '/press_emergency')
        self.emergency_release_client = self.create_client(Trigger, '/release_emergency')

    def door_callback(self, msg):
        self.door_closed = msg.data

    def emergency_callback(self, msg):
        self.emergency_pressed = msg.data

    def stack_callback(self, msg):
        self.stack_status = msg.data

    def barcode_callback(self, msg):
        self.barcode = msg.data

    def call_setbool(self, client, value):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().message

    def call_trigger(self, client):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().message

hmi_node = HMINode()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/status')
def status():
    return jsonify({
        'door': "Closed" if hmi_node.door_closed else "Open",
        'emergency': "Pressed" if hmi_node.emergency_pressed else "Released",
        'stack': hmi_node.stack_status,
        'barcode': hmi_node.barcode
    })

@app.route('/toggle-door', methods=['POST'])
def toggle_door():
    new_state = not hmi_node.door_closed
    result = hmi_node.call_setbool(hmi_node.door_toggle_client, new_state)
    return jsonify({'result': result})

@app.route('/press-emergency', methods=['POST'])
def press_emergency():
    result = hmi_node.call_trigger(hmi_node.emergency_press_client)
    return jsonify({'result': result})

@app.route('/release-emergency', methods=['POST'])
def release_emergency():
    result = hmi_node.call_trigger(hmi_node.emergency_release_client)
    return jsonify({'result': result})

def ros_spin():
    rclpy.spin(hmi_node)

def flask_main():
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    threading.Thread(target=ros_spin, daemon=True).start()
    flask_main()
