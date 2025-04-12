from flask import Flask, request, jsonify
import requests
from flask_cors import CORS  # Add this line

app = Flask(__name__)
CORS(app)  # Enable CORS globally

@app.route('/pick', methods=['POST'])
def pick():
    data = request.get_json()
    print(f"[WMS] Pick request received: {data}")

    try:
        # Forward to robot client
        response = requests.post("http://localhost:8081/confirmPick", json=data)
        return jsonify(response.json())
    except Exception as e:
        return jsonify({
            "error": str(e)
        }), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
