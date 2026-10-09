# SmartFleet AI

Warehouse automation platform with a physics-backed 3D warehouse and an
autonomous storage robot. **Current state: Milestone 2A — Functional lift and
fork control** (on top of Milestone 1, the core foundation).

- **Physics:** PyBullet (DIRECT mode, fixed 1/240 s timestep) owns the robot's
  state. The robot moves only because its wheel motors drive it through contact
  friction. The lift and forks move only through force- and velocity-limited
  joint motors on their prismatic joints. There is no scripted or teleported
  pose or joint value.
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
braking work, and that reset is deterministic.

The lift and fork tests (`tests/test_mechanisms.py`, `tests/test_api_mechanisms.py`)
advance the physics and check measured joint states. They cover:
- monotonic motion at the configured speed limits;
- arrival at the target;
- rejection of out-of-range or malformed targets;
- joint limits holding even when validation is bypassed;
- Stop holding a mechanism mid-travel;
- a lift jammed under a rack shelf being stopped without tipping the robot;
- driving with the lift raised;
- the WebSocket streaming the intermediate joint positions.

If PyBullet is missing, the physics tests are reported as skipped and the rest
still run.

## Using the Robot Lab

1. The header shows **Backend** (connection), **Physics** (PyBullet ready) and
   the simulation state. The simulation always starts **STOPPED**: physics is
   frozen and no motion is requested.
2. Press **Start** to run physics in real time. **Pause** freezes it.
   **Reset** rebuilds the world with the robot back at its home pose (STOPPED).
3. **Manual drive** (only while running): hold ▲ ▼ ⟲ ⟳ or **W A S D** /
   arrow keys. Commands are re-sent while held and carry a 0.4 s dead-man
   timeout, so releasing (or losing the connection) stops the robot. It does
   not affect the lift or forks. **Space** or the **STOP** pad button is an
   e-stop: it brakes the chassis *and* holds the lift and forks. The sliders set
   speed and turn rate within the configured limits.
4. **Lift & forks** (only while running). Each mechanism card shows the
   *measured* position, the target, velocity and a state badge: HOLDING,
   RAISING/LOWERING, EXTENDING/RETRACTING, or BLOCKED. All of these come from
   PyBullet.
   - **Slider + "Move to"** sends an absolute target.
   - **− / + 5 cm** jogs the target. A jog past a travel limit saturates at
     the limit, and the event log flags it.
   - **Lower/Raise fully** and **Retract/Extend fully** go to the travel limits.
   - **Stop** holds the mechanism where it is.
   - **Lift presets** (Home, Shelf L1–L4, Stand) align the fork surface 2 cm
     above each real shelf or stand top.
   - **Stop lift & forks** holds both.

   The 3D robot moves only when the reported joint positions change.
5. **Telemetry** shows position, heading, measured linear and angular velocity
   (with the commanded values), wheel speeds, parking-brake state, lift and fork
   positions and the container pose, all from PyBullet.
6. **Viewport:** drag to orbit, right-drag to pan, scroll to zoom. **Overview**
   / **Top** set the camera; **Follow robot** keeps it centred.
7. The **Event log** combines server events with client events. Server events
   include initialization, start/pause/reset, drive and mechanism commands,
   rejected requests, and blocked mechanisms with the obstacle they touched.
   Client events cover the connection and errors.

### Trying the lift and forks

With the simulation running:
1. Click **Shelf L3**. The carriage climbs at 0.30 m/s until the fork surface
   reads about 1.32 m and the badge turns to HOLDING.
2. Click **Extend fully**. The forks run out to 0.60 m at 0.25 m/s.
3. Click **Raise fully**, then **Stop** after a second. The lift holds at
   the position where it stopped.
4. Try a jog past a limit: at 1.60 m, click **+ 5 cm**. The target stays
   at 1.60 m and the event log notes that it was clamped.
5. Collision check:
   1. Drive the robot until it faces a rack, close enough that the extended
      forks reach under a shelf.
   2. Raise the lift. The forks stop against the shelf's underside, the badge
      shows BLOCKED, and the event log names the shelf (e.g. `C2/shelf-1`).
   3. Lower the lift to clear the fault.
6. **Pause** or **STOP** (Space) holds both mechanisms. **Reset** returns them
   to their default positions.

## API

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/health` | Liveness |
| GET | `/api/ready` | Readiness (200 when physics is initialized and the robot model is validated, otherwise 503 with a reason) |
| GET | `/api/config/warehouse` | Canonical config, derived collision boxes, robot model, lift presets |
| GET | `/api/simulation/state` | Current snapshot (status, sim time, robot and container state) |
| POST | `/api/simulation/start` / `pause` / `reset` | Simulation control (pause also holds lift and forks) |
| POST | `/api/robot/velocity` | `{"linear": m/s, "angular": rad/s, "duration": s}`, bounded (±1.0 m/s, ±1.5 rad/s, 0.05–2.0 s); 409 unless running, 422 if malformed |
| POST | `/api/robot/stop` | Cancel the drive request and brake (lift and forks unaffected) |
| POST | `/api/robot/estop` | Stop all motion: brake the chassis and hold lift and forks (any state) |
| POST | `/api/robot/lift/target` | `{"position": m}`, 0–1.60; 422 outside the limits, 409 unless running |
| POST | `/api/robot/lift/jog` | `{"delta": m}`, non-zero, within ±0.25; the target saturates at the travel limit |
| POST | `/api/robot/lift/stop` | Hold the lift at its measured position (any state) |
| POST | `/api/robot/forks/target` | `{"position": m}`, 0–0.60 |
| POST | `/api/robot/forks/jog` | `{"delta": m}`, non-zero, within ±0.20 |
| POST | `/api/robot/forks/stop` | Hold the forks at their measured position |
| POST | `/api/robot/mechanisms/stop` | Hold both lift and forks |
| WS | `/ws/telemetry` | Snapshots at 30 Hz plus new events |

Each mechanism in the telemetry (`robot.lift`, `robot.forks`) reports these
values, all measured by PyBullet:
- `position` and `velocity`
- `target`, `error` and `at_target`
- `state`: `holding`, `moving` or `blocked`
- `fault`: `stalled`, `overload` or `tilt`
- `applied_force`, plus its limits and default position

`robot.fork_surface_height` is read from the fork link's pose.

## Conventions and robot specification

- **World frame:** right-handed, Z up, metres/radians. The origin is the floor
  centre, +X points east (towards the docks) and +Y north. Yaw is
  counter-clockwise from +X.
- **Robot body frame:** +X forward, +Y left, origin at the drive-axle midpoint
  (the base centre of mass), so telemetry `z` is about one wheel radius.
- **Robot SF-1:** chassis 0.90 × 0.56 × 0.24 m, 87 kg total. Two driven wheels
  (r = 0.10 m, track 0.62 m) and two frictionless casters. Rear mast 1.92 m.
  Lift carriage is a prismatic joint, 0–1.60 m (fork surface 0.34–1.94 m above
  the floor), with a 450 N motor at 0.30 m/s. Telescopic forks are a prismatic
  joint, 0–0.60 m, with a 300 N motor at 0.25 m/s. Retracted forks stay inside
  the footprint.
- **Lift/fork protection:** a mechanism that can't reach its target is
  stopped where it is and reported as `blocked` (with the obstacle it touched)
  in any of three cases:
  - it stalls for 0.5 s;
  - its motor sits at its force limit for 0.1 s (overload);
  - the chassis tilts more than 0.05 rad while it moves.

  The motor ratings are deliberately modest. A 2000 N lift jammed under a
  shelf was found to lever the 87 kg robot 26° off the floor.
- **Motion limits:** 1.0 m/s, 1.5 rad/s, accelerating at 1.0 m/s² and 3 rad/s²,
  braking at 2.0 m/s² and 5 rad/s². A parking brake holds the wheels once the
  robot is at rest.
- **Warehouse:** 24 × 16 m floor, 8 four-level tote racks, loading and delivery
  zones with stands, one sample tote on the inbound stand.

Every value lives in `config/warehouse.json`. The URDF is generated from it at
runtime, and the frontend draws the robot and racks from the API, so the layers
cannot drift apart.

## Known limitations (Milestone 2A)

- The lift and forks move physically, but there is **no load-handling
  workflow** yet. Nothing is picked up, carried or stored, and the sample tote
  isn't attached to the forks.
- There's no drive interlock: the robot can drive with forks extended or the
  lift raised. The physics handles collisions, and the tilt protection stops
  mechanism motion, but nothing prevents the drive command.
- Manual drive is a temporary verification tool. There is no navigation,
  obstacle avoidance or collision prevention: you can drive into racks, and
  the physics will stop the robot.
- The backend runs one simulation instance shared by **all** connected
  browsers. Two people driving at once control the same robot.
- The 3D view renders the latest 30 Hz telemetry sample without interpolation,
  so motion can look slightly stepped.
- The frontend bundle is about 0.8 MB because Three.js isn't code-split yet.

## Next milestone (proposed: Milestone 2B)

Navigation foundation: an occupancy grid derived from the same static geometry,
an A* planner, and a path-following controller that drives the existing
differential-drive interface. Inventory, AI integration and the full
pickup-and-delivery workflow follow later.
