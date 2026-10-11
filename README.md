# SmartFleet AI: Autonomous Warehouse Robotics & Digital Twin

[![Build & Tests](https://img.shields.io/badge/tests-241%20passed-brightgreen.svg)]()
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)]()
[![Physics](https://img.shields.io/badge/physics-PyBullet%20DIRECT-orange.svg)]()
[![Frontend](https://img.shields.io/badge/frontend-React%20%7C%20Three.js%20%7C%20Vite-blueviolet.svg)]()
[![Deployment](https://img.shields.io/badge/deploy-Vercel-black.svg)]()

SmartFleet AI is an end-to-end autonomous mobile robot (AMR) warehouse automation platform and interactive 3D digital twin. The system models a high-fidelity warehouse environment in PyBullet physics (240 Hz fixed timestep), featuring an articulated differential-drive storage robot with a telescopic lift and forks, autonomous A* navigation, dynamic obstacle avoidance, priority task scheduling, RGB-D perception, and perception-guided closed-loop container pickup.

```
config/warehouse.json ──► simulation/ (PyBullet DIRECT world, 240 Hz physics)
  (Single Source of       │  • Diff-drive AMR kinematics & contact dynamics
       Truth)             │  • Prismatic lift mast & telescopic forks
                          │  • A* planner & dynamic obstacle avoidance
                          │  • Lift-mounted RGB-D camera & perception pipeline
                          │  • Closed-loop alignment & autonomous pickup
                          ▼
                     backend/ (FastAPI)
                          │  • REST API endpoints (/api/*)
                          │  • SQLite Task Queue persistence (/api/tasks/*)
                          │  • High-rate WebSocket telemetry (/ws/telemetry @ 30 Hz)
                          ▼
                     frontend/ (Robot Lab - React 19 + TypeScript + Vite + Three.js)
                          • Real-time 3D warehouse digital twin rendering
                          • Interactive manual & autonomous navigation controls
                          • Live RGB perception feed & object localization overlay
                          • Task scheduler management & telemetry inspection
```

---

## Completed Engineering Milestones

The project incorporates seven engineering milestones:

1. **Milestone 1 — Core Foundation & Physics Simulation**
   - Canonical warehouse configuration (`config/warehouse.json`) defining static collision geometry, racks, stands, and zones.
   - PyBullet DIRECT-mode physics simulation with rigid-body contact dynamics at a fixed 240 Hz timestep.
   - Articulated URDF mobile robot (differential-drive wheels, casters, chassis, rear mast, carriage, telescopic forks).
   - FastAPI server with health checks, configuration endpoints, and WebSocket telemetry streaming at 30 Hz.
   - React + Three.js Robot Lab frontend rendering real-time simulation state.

2. **Milestone 2A — Functional Lift & Fork Mechanism Control**
   - Physics-driven prismatic joint motors with force and velocity limits (lift: 450 N @ 0.30 m/s; forks: 300 N @ 0.25 m/s).
   - Bounded positioning, incremental jogs, travel-limit saturation, and stop/hold controls.
   - Safety monitoring: stall detection (0.5 s), motor overload protection (0.1 s), and chassis-tilt interlocks (0.05 rad).
   - Shelf height presets aligning forks 2 cm above rack levels (L1–L4).

3. **Milestone 2B — Autonomous Navigation & Path Following**
   - 2D warehouse occupancy grid automatically built from physical collision bounding boxes with robot safety inflation.
   - 8-directional A* grid path planning with octile distance heuristics.
   - Trajectory generator with curvature-bounded turning, smooth acceleration/deceleration profiles, and arrival tolerance.
   - Differential-drive path-following controller with dead-man safety timeout and parking brake engagement.

4. **Milestone 3 — Cargo Handling, Staging & Shelf Docking**
   - Physically simulated containers/totes with contact friction, mass properties, and stacking surfaces.
   - Container attachment and mechanical locking onto forks during transit.
   - Automated pick-and-place workflows: approach, mast alignment, fork extension, fork lift, retraction, and staging delivery.

5. **Milestone 4 — Dynamic Obstacle Detection & Avoidance**
   - Dynamic obstacle simulation with real-time velocity and trajectory tracking.
   - Proactive safety boundary (safety bubble) around the robot.
   - Fast reactive braking (< 0.5 s reaction time) when an obstacle breaches the safety zone.
   - Dynamic local replanning to route around unexpected obstructions without losing task state.

6. **Milestone 5 — Priority Task Scheduler & Persistence**
   - Task management service supporting priority queues (`HIGH`, `NORMAL`, `LOW`).
   - SQLite persistence layer (`tasks.db`) ensuring task state survives backend restarts.
   - Task lifecycle management: `PENDING`, `RUNNING`, `COMPLETED`, `FAILED`, `CANCELLED`.
   - Preemption handling, task cancellation, and execution audit history.

7. **Milestone 6 — Simulated Visual Perception Pipeline**
   - Lift-mounted simulated RGB-D camera tracking the carriage pose in real time.
   - Efficient 320x240 capture at a dedicated 5 Hz background thread safely decoupled from the 240 Hz physics loop.
   - Metric depth buffer transformation from PyBullet depth buffers using camera projection matrices.
   - PyBullet segmentation ID mapping paired with OpenCV contour and bounding box analysis.
   - 3D warehouse world coordinate reprojection for detected objects and AABB boundary distance error metrics.

8. **Milestone 7 — Perception-Guided Autonomous Pickup**
   - Closed-loop autonomous target acquisition: identifies container IDs and spatial positions from the perception pipeline.
   - Two-phase approach: coarse A* navigation to an observation waypoint followed by visual servoing.
   - Fine visual alignment: iterative lateral offset and heading angle correction facing the target container pocket.
   - Physical pickup sequencing: carriage adjustment to slot height, fork extension into the pocket, vertical lift, and retraction.
   - Multi-criteria pickup verification: physical contact point validation and vertical load displacement checks.
   - Telemetry integration and frontend perception overlay with real-time bounding boxes and state indicators.

---

## System Architecture

### Directory Layout

```
SmartFleet-AI/
├── config/
│   └── warehouse.json         # Single source of truth: layout, robot kinematics, limits
├── simulation/
│   ├── world.py               # PyBullet world, static geometry, object tracking
│   ├── robot.py               # URDF generation, drive kinematics, mechanism motors
│   ├── navigation.py          # Occupancy grid, A* planning, trajectory tracking
│   ├── perception.py          # RGB-D camera capture, metric depth, 3D reprojection
│   ├── pickup.py              # Perception-guided fine alignment & pickup controller
│   └── obstacles.py           # Dynamic obstacle simulation & safety zones
├── backend/
│   ├── app/
│   │   ├── main.py            # FastAPI application & route definitions
│   │   ├── service.py         # 240 Hz simulation background service & lock coordination
│   │   ├── tasks.py           # Task queue manager, priority scheduler, SQLite persistence
│   │   ├── schemas.py         # Pydantic request/response schemas
│   │   └── database.py        # SQLite database connection & schema migrations
│   └── requirements-dev.txt   # Backend Python dependencies
├── frontend/
│   ├── src/
│   │   ├── components/        # React UI panels (Controls, Viewport, Perception, Tasks)
│   │   ├── api/
│   │   │   └── client.ts      # REST and WebSocket client with configurable endpoints
│   │   ├── App.tsx            # Main Robot Lab application layout
│   │   └── main.tsx           # React entrypoint
│   ├── package.json           # Frontend dependencies & scripts
│   └── vite.config.ts         # Vite build configuration & local dev proxy
├── tests/
│   ├── test_kinematics.py     # Differential-drive kinematics tests
│   ├── test_navigation.py     # A* planner & trajectory tests
│   ├── test_mechanisms.py     # Lift and fork joint motor tests
│   ├── test_tasks.py          # Task queue & persistence tests
│   ├── test_perception.py     # RGB-D metric depth & reprojection tests
│   ├── test_pickup_alignment.py # Perception-guided pickup lifecycle tests
│   └── test_pickup_evaluation.py# Quantitative pickup accuracy benchmarks
├── vercel.json                # Vercel monorepo deployment configuration
└── README.md                  # System documentation
```

---

## Local Setup & Installation

### Prerequisites

| Component | Tested Version | Notes |
|---|---|---|
| OS | macOS / Linux / Windows 10/11 | Cross-platform |
| Python | 3.11 | In Conda environment (`smartfleet`) |
| PyBullet | 3.2.5 | From **conda-forge** |
| Node.js / npm | ≥ 20.19 or ≥ 22.12 / npm 10+ | Required for frontend |

> **Note on PyBullet Installation:**
> Always install PyBullet via `conda-forge` (`conda install -c conda-forge pybullet`). Compiling PyBullet directly via pip requires C++ build toolchains which may fail on some operating systems.

### 1. Backend Setup

```bash
# Clone the repository
git clone https://github.com/karthikbalaji1111-wq/SmartFleet-AI.git
cd SmartFleet-AI

# Create conda environment with PyBullet
conda create -n smartfleet -c conda-forge python=3.11 pybullet -y
conda activate smartfleet

# Install Python backend dependencies
python -m pip install -r backend/requirements-dev.txt
```

### 2. Frontend Setup

```bash
cd frontend
npm install
cd ..
```

---

## Running the Application

Start the backend and frontend in two separate terminals from the repository root:

### Terminal 1: Backend Server

```bash
conda activate smartfleet
python -m uvicorn backend.app.main:app --host 127.0.0.1 --port 8000
```

- Health check: [http://127.0.0.1:8000/api/ready](http://127.0.0.1:8000/api/ready)
- Interactive OpenAPI Docs: [http://127.0.0.1:8000/docs](http://127.0.0.1:8000/docs)

### Terminal 2: Frontend Client (Robot Lab)

```bash
cd frontend
npm run dev
```

- Open the dashboard at [http://127.0.0.1:5173](http://127.0.0.1:5173).
- The Vite development server automatically proxies `/api` and `/ws` to the local FastAPI backend.

---

## Automated Test Suites

The test suite validates kinematics, physics stability, safety limits, mechanism dynamics, perception accuracy, and autonomous pickup performance:

```bash
conda activate smartfleet

# Run full test suite (241 passed)
pytest

# Run Milestone 7 pickup alignment tests (17 passed)
pytest tests/test_pickup_alignment.py -v

# Run Milestone 7 quantitative evaluation benchmarks (5 passed)
pytest tests/test_pickup_evaluation.py -v

# Run visual perception pipeline tests
pytest tests/test_perception.py -v
```

### Test Coverage Highlights
- **Kinematics & Physics:** Differential-drive odometry, wheel slip, parking-brake holding, tilt protection.
- **Mechanisms:** Lift/fork travel bounds, force limits, stall/overload detection, preset alignment.
- **Navigation:** A* path optimality, trajectory tracking error < 5 cm, collision-free obstacle clearance.
- **Perception Pipeline:** Metric depth accuracy (< 1.5 cm error), 3D coordinate reprojection, AABB boundary distance.
- **Autonomous Pickup:** End-to-end target acquisition, lateral alignment within ±2.5 cm, orientation error < 0.05 rad, physical contact verification, post-lift load elevation checks.

---

## Frontend Deployment on Vercel

The frontend is configured for deployment to **Vercel** as a single-page application (SPA).

### Configuration Details (`vercel.json`)
The repository includes a root `vercel.json` configured for monorepo builds:
- **Build Command:** `cd frontend && npm install && npm run build`
- **Output Directory:** `frontend/dist`
- **Framework:** Vite
- **Routing:** SPA rewrites redirecting `/(.*)` to `/index.html`

### Connecting Frontend to Backend
The frontend connects to the backend using environment variables configured in `frontend/src/api/client.ts`:
- `VITE_BACKEND_URL`: Base HTTP URL for the FastAPI backend (e.g., `https://your-backend-service.com` or `http://127.0.0.1:8000`)
- `VITE_WS_URL`: WebSocket URL for real-time telemetry (e.g., `wss://your-backend-service.com/ws/telemetry` or `ws://127.0.0.1:8000/ws/telemetry`)

When running in local development mode without environment variables, the frontend defaults to `http://127.0.0.1:8000` and `ws://127.0.0.1:8000/ws/telemetry`.

> **Important Deployment Architecture Note:**
> Vercel hosts the static React + Three.js client-side application. The SmartFleet AI backend runs a continuous 240 Hz PyBullet physics simulation engine and persistent WebSockets, which cannot execute inside serverless functions.
>
> To use the deployed Vercel frontend with a live backend:
> 1. Run the backend locally and use a secure tunnel (e.g., `ngrok http 8000`), configuring the resulting HTTPS/WSS URLs in Vercel's environment variables (`VITE_BACKEND_URL` and `VITE_WS_URL`), OR
> 2. Deploy the backend Docker container to a continuous cloud platform supporting persistent processes and WebSockets (such as Render, Railway, Fly.io, or AWS EC2).

---

## REST API Summary

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/health` | System health and service status |
| `GET` | `/api/ready` | Readiness probe (verifies PyBullet physics and robot model) |
| `GET` | `/api/config/warehouse` | Canonical warehouse layout, rack positions, and robot limits |
| `GET` | `/api/simulation/state` | Current snapshot of robot pose, mechanisms, and simulation time |
| `POST` | `/api/simulation/start` | Start or resume real-time physics simulation |
| `POST` | `/api/simulation/pause` | Freeze simulation and hold mechanism states |
| `POST` | `/api/simulation/reset` | Reset simulation world, robot pose, and mechanisms to default |
| `POST` | `/api/robot/velocity` | Command chassis linear and angular velocity with dead-man timeout |
| `POST` | `/api/robot/stop` | Command active braking on chassis drive motors |
| `POST` | `/api/robot/estop` | Emergency stop: immediate chassis brake and mechanism lock |
| `POST` | `/api/robot/lift/target` | Command absolute carriage lift position (0.0 – 1.60 m) |
| `POST` | `/api/robot/forks/target` | Command absolute telescopic fork extension (0.0 – 0.60 m) |
| `POST` | `/api/robot/navigate` | Request autonomous navigation to target (x, y, yaw) coordinates |
| `POST` | `/api/robot/pickup` | Initiate perception-guided autonomous pickup sequence |
| `GET` | `/api/perception/state` | Retrieve latest camera metadata, detections, and 3D estimates |
| `GET` | `/api/tasks` | List all tasks with status and priority |
| `POST` | `/api/tasks` | Enqueue a new automated warehouse mission |
| `POST` | `/api/tasks/{id}/cancel`| Cancel a pending or running task |
| `WS` | `/ws/telemetry` | High-frequency telemetry stream (30 Hz) |

---

## License

This project is developed for educational and portfolio demonstration purposes as part of an advanced robotics software engineering curriculum.
