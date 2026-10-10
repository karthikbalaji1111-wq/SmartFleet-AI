import { useState } from 'react';
import type { WarehouseConfigResponse, SimulationSnapshot } from '../api/types';
import { api } from '../api/client';

interface Props {
  config: WarehouseConfigResponse | null;
  snapshot: SimulationSnapshot | null;
}

export function WarehouseOperationsPanel({ config, snapshot }: Props) {
  const [container, setContainer] = useState<string>('');
  const [destination, setDestination] = useState<string>('');
  const [type, setType] = useState<'storage' | 'retrieval'>('storage');
  const [priority, setPriority] = useState<number>(0);
  
  if (!config) return null;
  
  const containers = config.config.warehouse.containers;
  const stations = config.config.warehouse.stations;
  const slots = config.config.warehouse.slots || [];
  
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!container || !destination) return;
    try {
      await api.taskSubmit({ container_id: container, destination, type, priority });
      setContainer('');
      setDestination('');
    } catch (err) {
      console.error(err);
    }
  };

  const handleCancelActive = async () => {
    try {
      await api.taskCancel();
    } catch (err) {
      console.error(err);
    }
  };

  const handleCancelTask = async (taskId: string) => {
    try {
      await api.taskCancelSpecific(taskId);
    } catch (err) {
      console.error(err);
    }
  };

  const handleSetPriority = async (taskId: string, newPriority: number) => {
    try {
      await api.taskSetPriority(taskId, newPriority);
    } catch (err) {
      console.error(err);
    }
  };

  const tasks = snapshot?.tasks;

  return (
    <section className="panel">
      <h2 className="panel-title">Task Scheduler</h2>
      <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '8px', marginBottom: '16px' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between' }}>
          <label>Type:</label>
          <select value={type} onChange={(e) => setType(e.target.value as 'storage' | 'retrieval')}>
            <option value="storage">Storage</option>
            <option value="retrieval">Retrieval</option>
          </select>
        </div>
        <div style={{ display: 'flex', justifyContent: 'space-between' }}>
          <label>Container:</label>
          <select value={container} onChange={(e) => setContainer(e.target.value)}>
            <option value="">--Select--</option>
            {containers.map(c => <option key={c.id} value={c.id}>{c.id}</option>)}
          </select>
        </div>
        <div style={{ display: 'flex', justifyContent: 'space-between' }}>
          <label>Destination:</label>
          <select value={destination} onChange={(e) => setDestination(e.target.value)}>
            <option value="">--Select--</option>
            <optgroup label="Stations">
              {stations.map(s => <option key={s.id} value={s.id}>{s.id}</option>)}
            </optgroup>
            <optgroup label="Slots">
              {slots.map((s: any) => <option key={s.id} value={s.id}>{s.id}</option>)}
            </optgroup>
          </select>
        </div>
        <div style={{ display: 'flex', justifyContent: 'space-between' }}>
          <label>Priority:</label>
          <input type="number" value={priority} onChange={(e) => setPriority(parseInt(e.target.value) || 0)} style={{ width: '80px' }} />
        </div>
        <div style={{ display: 'flex', gap: '8px', marginTop: '8px' }}>
          <button type="submit" disabled={!container || !destination}>Submit Task</button>
          <button type="button" onClick={handleCancelActive} disabled={!tasks?.active_task}>Cancel Active</button>
        </div>
      </form>

      {tasks && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '8px', fontSize: '12px' }}>
          <div>
            <strong>Active Task: </strong> 
            {tasks.active_task ? (
              <span>
                {tasks.active_task.container_id} ➔ {tasks.active_task.destination} (Pri: {tasks.active_task.priority})
              </span>
            ) : 'None'}
          </div>
          <div>
            <strong>Queue ({tasks.queue.length})</strong>
            {tasks.queue.map(t => (
              <div key={t.id} style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginTop: '4px', padding: '4px', border: '1px solid #ccc' }}>
                <span>{t.type} {t.container_id} ➔ {t.destination}</span>
                <div style={{ display: 'flex', gap: '4px' }}>
                  <input type="number" value={t.priority} onChange={(e) => handleSetPriority(t.id, parseInt(e.target.value) || 0)} style={{ width: '40px', fontSize: '10px' }} />
                  <button type="button" onClick={() => handleCancelTask(t.id)} style={{ fontSize: '10px', padding: '2px' }}>X</button>
                </div>
              </div>
            ))}
          </div>
          <div>
            <strong>History (Last 5)</strong>
            {tasks.history.slice(-5).map(t => (
              <div key={t.id} style={{ display: 'flex', justifyContent: 'space-between', marginTop: '2px', color: t.status === 'completed' ? 'green' : (t.status === 'failed' || t.status === 'cancelled' ? 'red' : 'black') }}>
                <span>{t.container_id} ➔ {t.destination}</span>
                <span>{t.status}</span>
              </div>
            ))}
          </div>
        </div>
      )}
    </section>
  );
}
