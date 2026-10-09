# SmartFleet AI

Warehouse automation platform with a physics-backed 3D warehouse and an
autonomous storage robot. **Current state: Milestone 1 — Core Foundation.**

- **Physics:** PyBullet (DIRECT mode, fixed 1/240 s timestep) owns the robot's
  state. The robot moves only because its wheel motors drive it through contact
  friction. There is no scripted or teleported pose.
- **Backend:** FastAPI serves the canonical config, simulation control, bounded
  drive commands and a WebSocket telemetry stream.
- **Frontend (Robot Lab):** React + TypeScript + Vite + Three.js. It renders
  exactly what the backend reports: the robot's pose and joint positions, and
  the warehouse built from the same collision boxes PyBullet uses.

```
config/warehouse.json ──► simulation/ (PyBullet world, URDF robot, diff-drive)
        (single source        │  state read from PyBullet only
         of truth)            ▼
                         backend/ (FastAPI)  ── REST /api/*  ──►  frontend/
                                             ── WS /ws/telemetry ─►  (Three.js)
```

## Repository layout

| Path | Contents |
|---|---|
| `config/warehouse.json` | Canonical warehouse layout, robot dimensions/limits, simulation parameters |
| `simulation/` | Config models, static collision geometry, URDF generation, differential-drive kinematics, PyBullet world |
| `backend/app/` | FastAPI app (`main.py`), simulation service and loop (`service.py`), API schemas |
| `frontend/` | Robot Lab UI (React, TypeScript, Vite, Three.js) |
| `tests/` | Pytest suite (config, kinematics, robot model, physics motion, reset, API) |

## Requirements (tested versions)

| Component | Version |
|---|---|
| Windows | 10/11 |
| Miniforge (conda) | any recent |
| Python | 3.11 (conda env `smartfleet`) |
| PyBullet | 3.2.5 from **conda-forge** |
| FastAPI / Starlette / Pydantic | 0.143.0 / 1.7.0 / 2.14.0 |
| Uvicorn / websockets | 0.54.0 / 17.2 |
| Pytest / httpx | 9.1.1 / 0.28.1 |
| Node.js / npm | ≥ 20.19 or ≥ 22.12 (tested 24.17) / 11 |
| React / Three.js | 19.3.0 / 0.186.1 |
| Vite / TypeScript | 8.3.1 / 6.0.3 |

> **Why conda for PyBullet?** PyPI ships **no Windows wheels** for PyBullet, so
> `pip install pybullet` tries to compile it and fails without the Visual C++
> Build Tools. conda-forge provides a prebuilt Windows package.

## Installation (Windows)

1. **Install Miniforge** from <https://github.com/conda-forge/miniforge> (or
   `winget install CondaForge.Miniforge3`), then open the **Miniforge Prompt**.

2. **Get the code**

   ```bat
   git clone https://github.com/karthikbalaji1111-wq/SmartFleet-AI.git
   cd SmartFleet-AI
   ```

3. **Create the Python environment** (once):

   ```bat
   conda create -n smartfleet -c conda-forge python=3.11 pybullet
   conda activate smartfleet
   python -m pip install -r backend\requirements-dev.txt
   ```

   pip sees PyBullet as already installed and does not try to compile it.

4. **Install the frontend** (once):

   ```bat
   cd frontend
   npm install
   cd ..
   ```

## Running (two terminals)

Run both from the repository root.

**Terminal 1: backend** (Miniforge Prompt):

```bat
conda activate smartfleet
python -m uvicorn backend.app.main:app --host 127.0.0.1 --port 8000
```

If plain PowerShell can't run `conda activate`, call the environment's Python
directly:

```powershell
& "$env:USERPROFILE\miniforge3\envs\smartfleet\python.exe" -m uvicorn backend.app.main:app --host 127.0.0.1 --port 8000
```

Check it: <http://127.0.0.1:8000/api/ready> should return `"ready": true`.
Interactive API docs are at <http://127.0.0.1:8000/docs>.

**Terminal 2: frontend:**

```bat
cd frontend
npm run dev
```

Open <http://127.0.0.1:5173>. Vite proxies `/api` and `/ws` to the backend,
so no CORS setup is needed.

Production build: `npm run build` (type-checks, then bundles into `frontend/dist`).

## Tests

```bat
conda activate smartfleet
python -m pytest
```

The suite drives the real physics. For example, it checks that a 0.5 m/s
request for 3 s moves the robot about 1.37 m along its heading, that in-place
turns rotate without translating, that per-step motion is continuous (no
teleporting) and explained by wheel rotation, that the dead-man timeout and
braking work, and that reset is deterministic. If PyBullet is missing, the
physics tests are reported as skipped and the rest still run.

## Using the Robot Lab

1. The header shows **Backend** (connection), **Physics** (PyBullet ready) and
   the simulation state. The simulation always starts **STOPPED**: physics is
   frozen and no motion is requested.
2. Press **Start** to run physics in real time. **Pause** freezes it.
   **Reset** rebuilds the world with the robot back at its home pose (STOPPED).
3. **Manual drive** (only while running): hold ▲ ▼ ⟲ ⟳ or **W A S D** /
   arrow keys. Commands are re-sent while held and carry a 0.4 s dead-man
   timeout, so releasing (or losing the connection) stops the robot. **Space**
   or **STOP** brakes along the deceleration profile. The sliders set speed and
   turn rate within the configured limits.
4. **Telemetry** shows position, heading, measured linear and angular velocity
   (with the commanded values), wheel speeds, parking-brake state, lift height,
   fork extension and the container pose, all from PyBullet.
5. **Viewport:** drag to orbit, right-drag to pan, scroll to zoom. **Overview**
   / **Top** set the camera; **Follow robot** keeps it centred.
6. The **Event log** combines server events (initialization, start/pause/reset,
   commands, rejected requests) with client events (connection, errors).

## API

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/health` | Liveness |
| GET | `/api/ready` | Readiness (200 when physics is initialized and the robot model is validated, otherwise 503 with a reason) |
| GET | `/api/config/warehouse` | Canonical config, derived collision boxes, robot model |
| GET | `/api/simulation/state` | Current snapshot (status, sim time, robot and container state) |
| POST | `/api/simulation/start` / `pause` / `reset` | Simulation control |
| POST | `/api/robot/velocity` | `{"linear": m/s, "angular": rad/s, "duration": s}`, bounded (±1.0 m/s, ±1.5 rad/s, 0.05–2.0 s); 409 unless running, 422 if malformed |
| POST | `/api/robot/stop` | Cancel the request and brake |
| WS | `/ws/telemetry` | Snapshots at 30 Hz plus new events |

## Conventions and robot specification

- **World frame:** right-handed, Z up, metres/radians. The origin is the floor
  centre, +X points east (towards the docks) and +Y north. Yaw is
  counter-clockwise from +X.
- **Robot body frame:** +X forward, +Y left, origin at the drive-axle midpoint
  (the base centre of mass), so telemetry `z` is about one wheel radius.
- **Robot SF-1:** chassis 0.90 × 0.56 × 0.24 m, 87 kg total. Two driven wheels
  (r = 0.10 m, track 0.62 m) and two frictionless casters. Rear mast 1.92 m.
  Lift carriage is a prismatic joint, 0–1.60 m (fork surface 0.34–1.94 m above
  the floor). Telescopic forks are a prismatic joint, 0–0.60 m. Retracted
  forks stay inside the footprint.
- **Motion limits:** 1.0 m/s, 1.5 rad/s, accelerating at 1.0 m/s² and 3 rad/s²,
  braking at 2.0 m/s² and 5 rad/s². A parking brake holds the wheels once the
  robot is at rest.
- **Warehouse:** 24 × 16 m floor, 8 four-level tote racks, loading and delivery
  zones with stands, one sample tote on the inbound stand.

Every value lives in `config/warehouse.json`. The URDF is generated from it at
runtime, and the frontend draws the robot and racks from the API, so the layers
cannot drift apart.

## Known limitations (Milestone 1)

- Lift and forks are fully modelled, physical prismatic joints with limits and
  telemetry, but they are **held at their lower limits**. Commanded lift/fork
  motion is deferred, and nothing is picked or stored yet.
- Manual drive is a temporary verification tool. There is no navigation,
  obstacle avoidance or collision prevention: you can drive into racks, and
  the physics will stop the robot.
- The backend runs one simulation instance shared by all connected browsers.
- The 3D view renders the latest 30 Hz telemetry sample without interpolation,
  so motion can look slightly stepped.
- The frontend bundle is about 0.8 MB because Three.js isn't code-split yet.

## Next milestone (proposed: Milestone 2)

Physical lift and fork control (bounded position commands, tested against
limits), plus a navigation foundation: an occupancy grid derived from the same
static geometry, an A* planner and a path-following controller driving the
existing differential-drive interface. Inventory, AI integration and the full
pickup-and-delivery workflow follow later.
