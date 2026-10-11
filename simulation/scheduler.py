import sqlite3
import time
import uuid
from typing import Literal, List, Optional, Any
from pydantic import BaseModel
from .tasks import TaskManager, TaskRequest, TaskState, PickupTelemetry

TaskType = Literal["storage", "retrieval"]
TaskStatus = Literal["queued", "running", "completed", "failed", "cancelled"]

class TaskRecord(BaseModel):
    id: str
    type: TaskType
    priority: int
    container_id: str
    destination: str
    status: TaskStatus
    created_at: float
    error: Optional[str] = None
    phase: Optional[TaskState] = None

class TaskTelemetry(BaseModel):
    active_task: Optional[TaskRecord]
    queue: List[TaskRecord]
    history: List[TaskRecord]
    estop_active: bool = False
    pickup: Optional[PickupTelemetry] = None

class TaskScheduler:
    def __init__(self, world, db_path="tasks.db"):
        self.world = world
        self.task_manager = TaskManager(world)
        self.db = sqlite3.connect(db_path, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self._init_db()
        self.active_task: Optional[TaskRecord] = None
        self.estop_active = False
        self._recover()

    def _init_db(self):
        with self.db:
            self.db.execute("""
                CREATE TABLE IF NOT EXISTS tasks (
                    id TEXT PRIMARY KEY,
                    type TEXT,
                    priority INTEGER,
                    container_id TEXT,
                    destination TEXT,
                    status TEXT,
                    created_at REAL,
                    error TEXT
                )
            """)
            
    def _recover(self):
        # On startup, mark any 'running' tasks as 'failed' (interrupted)
        with self.db:
            self.db.execute("UPDATE tasks SET status = 'failed', error = 'Interrupted by restart' WHERE status = 'running'")

    def submit(self, request: TaskRequest, task_type: TaskType = "storage", priority: int = 0) -> str:
        # Validate task
        if request.container_id not in self.world.container_ids:
            raise ValueError(f"Unknown container {request.container_id}")
            
        dest_loc = self.task_manager._find_location(request.destination)
        if not dest_loc:
            raise ValueError(f"Unknown destination {request.destination}")

        # Check reservations
        # Queued or running tasks that target the same container or destination
        with self.db:
            cur = self.db.cursor()
            cur.execute("SELECT id, type, container_id, destination FROM tasks WHERE status IN ('queued', 'running')")
            for row in cur.fetchall():
                if row['container_id'] == request.container_id:
                    raise ValueError(f"Container {request.container_id} is already reserved by task {row['id']}")
                if row['destination'] == request.destination and request.destination != "inbound" and request.destination != "outbound":
                    raise ValueError(f"Destination {request.destination} is already reserved by task {row['id']}")
        
        # Check physical occupancy
        for cid, loc_id in getattr(self.world, 'container_locations', {}).items():
            if cid != request.container_id and loc_id == request.destination:
                raise ValueError(f"Destination {request.destination} is already occupied by {cid}")

        task_id = str(uuid.uuid4())
        created_at = time.time()
        with self.db:
            self.db.execute("""
                INSERT INTO tasks (id, type, priority, container_id, destination, status, created_at)
                VALUES (?, ?, ?, ?, ?, 'queued', ?)
            """, (task_id, task_type, priority, request.container_id, request.destination, created_at))
            
        return task_id

    def cancel(self, task_id: str = None):
        if task_id:
            with self.db:
                cur = self.db.cursor()
                cur.execute("SELECT status FROM tasks WHERE id = ?", (task_id,))
                row = cur.fetchone()
                if not row:
                    raise ValueError(f"Unknown task {task_id}")
                if row['status'] == 'queued':
                    self.db.execute("UPDATE tasks SET status = 'cancelled' WHERE id = ?", (task_id,))
                    return
                elif row['status'] == 'running':
                    if self.active_task and self.active_task.id == task_id:
                        self.task_manager.cancel()
                        self.db.execute("UPDATE tasks SET status = 'cancelled' WHERE id = ?", (task_id,))
                        self.active_task = None
                    return
                else:
                    raise ValueError(f"Cannot cancel task in state {row['status']}")
        else:
            # cancel active task
            if self.active_task:
                self.cancel(self.active_task.id)

    def change_priority(self, task_id: str, new_priority: int):
        with self.db:
            cur = self.db.cursor()
            cur.execute("SELECT status FROM tasks WHERE id = ?", (task_id,))
            row = cur.fetchone()
            if not row:
                raise ValueError(f"Unknown task {task_id}")
            if row['status'] != 'queued':
                raise ValueError("Can only change priority of queued tasks")
            self.db.execute("UPDATE tasks SET priority = ? WHERE id = ?", (new_priority, task_id))

    def tick(self, dt: float):
        self.task_manager.tick(dt)
        
        if self.active_task:
            state = self.task_manager.state
            self.active_task.phase = state
            if state in ("completed", "failed"):
                status = state
                error = getattr(self.task_manager, 'error', None)
                with self.db:
                    self.db.execute("UPDATE tasks SET status = ?, error = ? WHERE id = ?", (status, error, self.active_task.id))
                self.active_task = None
                
        # Handle estop
        if getattr(self, 'estop_active', False):
            # estop active, do not schedule new tasks
            return

        if not self.active_task and self.task_manager.state == "idle":
            # fetch next
            with self.db:
                cur = self.db.cursor()
                cur.execute("SELECT * FROM tasks WHERE status = 'queued' ORDER BY priority DESC, created_at ASC LIMIT 1")
                row = cur.fetchone()
                if row:
                    self.db.execute("UPDATE tasks SET status = 'running' WHERE id = ?", (row['id'],))
                    self.active_task = TaskRecord(**dict(row))
                    self.active_task.status = 'running'
                    self.task_manager.submit(TaskRequest(container_id=row['container_id'], destination=row['destination']))
                    
    def telemetry(self) -> TaskTelemetry:
        with self.db:
            cur = self.db.cursor()
            cur.execute("SELECT * FROM tasks WHERE status = 'queued' ORDER BY priority DESC, created_at ASC")
            queue = [TaskRecord(**dict(row)) for row in cur.fetchall()]
            
            cur.execute("SELECT * FROM tasks WHERE status IN ('completed', 'failed', 'cancelled') ORDER BY created_at DESC LIMIT 10")
            history = [TaskRecord(**dict(row)) for row in cur.fetchall()]
            
        return TaskTelemetry(
            active_task=self.active_task,
            queue=queue,
            history=history,
            estop_active=self.estop_active,
            pickup=self.task_manager.pickup_telemetry_data,
        )

    def set_estop(self, active: bool):
        self.estop_active = active
        if active and self.active_task:
            self.cancel(self.active_task.id)
