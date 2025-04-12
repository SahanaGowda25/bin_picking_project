# Bin Picking Cell Control – ROS2 + Docker + Flask + HMI (macOS)

## Project Overview
This project simulates a bin picking cell control system using ROS2 (Humble), containerized with Docker, and integrated with:
- Flask-based REST APIs
- A web-based Human-Machine Interface (HMI)
- ROS2 nodes representing physical components

The system processes pick requests from the user interface and simulates barcode scanning, door handle status, emergency button, and stack light behavior in real-time. It is designed to reflect real-world industrial setups with modular and scalable components.

---

## Technologies Used (on macOS)
| Component        | Tool / Framework      | Rationale                                      |
|------------------|------------------------|------------------------------------------------|
| ROS2 Nodes       | ROS2 Humble            | Long-term support, actively maintained, scalable |
| API Server       | Flask + flask-cors     | Lightweight and effective for REST APIs; CORS resolves browser-to-Docker issues |
| Frontend (HMI)   | HTML + JavaScript      | Simple, fast-loading GUI; avoids unnecessary frontend complexity |
| Networking       | requests, ROS2 services| Native communication between REST endpoints and ROS logic |
| Containerization | Docker + Compose       | Ensures portability, reproducibility, and platform independence, particularly useful for macOS where native ROS2 is not straightforward |
| Version Control  | Git (local)            | Tracks all development progress and iterations |

---

## Setup & Installation (macOS)

### Prerequisites
- Docker Desktop for macOS
- Python 3.11+ (installed inside Docker)
- Visual Studio Code 
- Use of Terminal app with zsh shell (default on macOS)

---

## Running the Project

### Step 1: Build Docker container
```bash
cd bin_picking_project/ros2_ws
docker-compose build
```

### Step 2: Launch container
```bash
docker-compose up -d
docker exec -it bin-picking-ros2 bash
```

### Step 3: Launch components in separate terminals

#### Terminal 1: ROS2 Nodes
```bash
cd /ros2_ws
./launch_ros.sh
```

#### Terminal 2: Flask APIs (WMS + Robot Client)
```bash
./launch_api.sh
```

#### Terminal 3: HMI Dashboard
```bash
./launch_hmi.sh
```

---

### Step 4: Open HMI Dashboard (macOS browser)
Open Safari or Chrome and visit:
```
http://localhost:5000
```

Interact with:
- Emergency button
- Door toggle
- Pick request and response messages
- Realtime barcode and stack light updates

If communication fails due to Docker isolation, replace `localhost` with your Mac IP address in `script.js`.

---

## Key Features
- Simulated barcode scanning every 2 seconds
- Door and emergency toggle via service interfaces
- Stack light updates based on system state
- REST APIs linked with ROS services for real-time data flow
- Pick request functionality integrated into HMI interface

---

## Demo Video
See `submission_video.mp4` for a screen-recorded walkthrough covering:
- Project launch
- GUI usage
- Pick request processing
- Emergency and door status interaction

---

## Folder Structure
```
bin_picking_project/
├── ros2_ws/
│   ├── api/              # Flask APIs
│   ├── hmi/              # HMI GUI (Flask + HTML + JS)
│   ├── src/              # ROS2 packages
│   ├── launch_*.sh       # Bash launchers
│   ├── Dockerfile
│   ├── docker-compose.yml
├── .git/                 # Local Git repo
├── submission_video.mp4
├── README.md
```

---

## External Resources and Libraries
| Resource         | Justification                          |
|------------------|----------------------------------------|
| flask-cors       | Required to resolve CORS issues when browser interacts with Flask in Docker |
| ROS2 documentation   | Guided ROS node/service setup     |
| requests library | Enables communication between WMS and robot client Flask apps |

---

## Design Decisions and Justifications

### Docker over Native Install (macOS)
Docker ensures a consistent environment on macOS where native ROS2 setup is complex. It simplifies the workflow and avoids system dependency conflicts.

### ROS2 Humble
Selected due to its long-term support status, community adoption, and because ROS Noetic (ROS1) is nearing end-of-life.

### Asynchronous Patterns in ROS2 Services
Asynchronous patterns allow for non-blocking service calls, which is important in robotics where multiple services may be running concurrently. This helps prevent delays or timeouts when waiting for responses and maintains responsiveness in the robot client.

### GUI Interface (HMI)
The HMI was implemented as a web interface using HTML and JavaScript served by Flask. This was chosen for:
- Compatibility with macOS browsers (Safari, Chrome)
- Accessibility without requiring additional frameworks
- Avoidance of heavier frameworks like React, simplifying deployment

### Frontend Communication (macOS Docker)
To enable the browser-based HMI on macOS to communicate with the Flask server inside Docker, either `host.docker.internal` or the local Mac IP address was used. Additionally, CORS headers were enabled using `flask-cors` to allow cross-origin requests from browser to Docker container.

---

## Final Checklist
- ROS2 nodes tested and fully functional
- All Flask APIs accessible and validated
- HMI serves complete real-time data
- Git version control included
- Demo video added
- Full reproducibility using Docker (macOS verified)

---

## Developer
Name: Sahana Chikkabelavanagala Krishnamurthy 


