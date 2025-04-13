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
- Python 3.11+ 
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
- Simulated barcode scanning every 5 seconds
- Door and emergency toggle via service interfaces
- Stack light updates based on system state
- REST APIs linked with ROS services for real-time data flow
- Pick request functionality integrated into HMI interface

---

## Demo Video
Access the drive link - https://drive.google.com/drive/folders/1nRz-rUeN1BBnJ1xj9TLPDi0J3FY3UK3r?usp=drive_link for a screen-recorded walkthrough covering:
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

### Asynchronous vs. Synchronous Use
For the ROS2 service-client architecture (e.g., toggling door state, emergency button), asynchronous methods were used where appropriate to ensure non-blocking behavior and enable multiple concurrent node operations. This helps achieve real-time responsiveness and better control across nodes handling sensor/actuator input.

However, the REST API layer implemented with Flask follows synchronous request-response cycles. This was chosen to maintain simplicity and better control over logic flow, which is acceptable given the low-load and straightforward interface design.

### GUI Interface (HMI)
The HMI was implemented as a lightweight GUI using Flask to serve static HTML and JavaScript. Reasons for this include:
- Seamless compatibility with macOS browsers (Safari, Chrome)
- No need for complex JavaScript frameworks (e.g., React or Angular)
- Fast load times and minimal setup
- Direct integration with backend Flask APIs for controlling and visualizing system state

### Frontend Communication (macOS Docker)
To enable the browser-based HMI on macOS to communicate with the Flask server inside Docker:
- `host.docker.internal` was used (preferred for Docker on macOS)
- As a fallback, Mac’s local IP address can be substituted in `script.js`
- Cross-origin issues were resolved with `flask-cors` to ensure smooth data flow between browser and Flask API endpoints

---

## Final Checklist
- ROS2 nodes tested and fully functional
- All Flask APIs accessible and validated
- HMI serves complete real-time data
- Git version control included
- Demo video added
- Full reproducibility using Docker (macOS verified)

---



## Summary of Changes Made
- Integrated Docker to run ROS2 and Python seamlessly on macOS
- Used ROS2 Humble to comply with current standards and avoid ROS1 deprecation
- Implemented ROS2 nodes for barcode scanner, door handle, emergency button, and stack light
- Built REST APIs using Flask to handle pick requests and simulate robot client behavior
- Used `flask-cors` to resolve cross-origin browser-to-Docker communication issues
- Created a lightweight web-based GUI (HMI) using HTML and JavaScript
- Used asynchronous service calls for ROS2 nodes and synchronous logic for Flask REST API
- Used `host.docker.internal` and IP fallback strategy for network bridging
- Structured system into reusable launch scripts for clear terminal separation
- Provided a demo video, Git repository, and complete submission ZIP

---
## Author
Sahana Chikkabelavangala Krishnamurthy 

