import pytest
from simulation.scheduler import TaskScheduler
from simulation.tasks import TaskRequest
from backend.app.schemas import TaskTelemetry
import pybullet as pb

@pytest.fixture
def scheduler(world):
    s = TaskScheduler(world, db_path=":memory:")
    # Initialize some containers if needed?
    return s

def test_priority_ordering(scheduler):
    # submit priority 1
    scheduler.submit(TaskRequest(container_id="TOTE-0001", destination="A1-B1-L2-S1"), priority=1)
    # submit priority 2
    scheduler.submit(TaskRequest(container_id="TOTE-0002", destination="A1-B1-L2-S2"), priority=2)
    # wait a bit for it to pick up
    scheduler.tick(0.1)
    
    # Priority 2 should be active
    assert scheduler.active_task.container_id == "TOTE-0002"

def test_fifo_ordering(scheduler):
    scheduler.submit(TaskRequest(container_id="TOTE-0001", destination="A1-B1-L2-S1"), priority=1)
    scheduler.submit(TaskRequest(container_id="TOTE-0002", destination="A1-B1-L2-S2"), priority=1)
    
    scheduler.tick(0.1)
    
    assert scheduler.active_task.container_id == "TOTE-0001"

def test_rejection_of_invalid_tasks(scheduler):
    with pytest.raises(ValueError, match="Unknown container"):
        scheduler.submit(TaskRequest(container_id="UNKNOWN", destination="A1-B1-L2-S1"))

def test_prevention_of_duplicate_reservations(scheduler):
    scheduler.submit(TaskRequest(container_id="TOTE-0001", destination="A1-B1-L2-S1"))
    with pytest.raises(ValueError, match="already reserved"):
        scheduler.submit(TaskRequest(container_id="TOTE-0001", destination="A1-B1-L2-S2"))

def test_prevention_of_conflicting_destinations(scheduler):
    scheduler.submit(TaskRequest(container_id="TOTE-0001", destination="A1-B1-L2-S1"))
    with pytest.raises(ValueError, match="already reserved"):
        scheduler.submit(TaskRequest(container_id="TOTE-0002", destination="A1-B1-L2-S1"))

def test_safe_cancellation_of_queued_tasks(scheduler):
    task1 = scheduler.submit(TaskRequest(container_id="TOTE-0001", destination="A1-B1-L2-S1"))
    task2 = scheduler.submit(TaskRequest(container_id="TOTE-0002", destination="A1-B1-L2-S2"))
    
    scheduler.cancel(task2)
    t = scheduler.telemetry()
    # It should not be in queue anymore (or marked cancelled)
    q_ids = [t.id for t in t.queue]
    assert task2 not in q_ids
    assert any(h.id == task2 and h.status == 'cancelled' for h in t.history)

def test_estop_interaction(scheduler):
    scheduler.submit(TaskRequest(container_id="TOTE-0001", destination="A1-B1-L2-S1"))
    scheduler.set_estop(True)
    scheduler.tick(0.1)
    
    # Active task should have been cancelled by set_estop
    assert scheduler.active_task is None

