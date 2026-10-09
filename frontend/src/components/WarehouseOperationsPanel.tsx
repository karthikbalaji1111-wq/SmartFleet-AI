import { useState } from 'react';
import type { WarehouseConfigResponse } from '../api/types';
import { api } from '../api/client';

interface Props {
  config: WarehouseConfigResponse | null;
}

export function WarehouseOperationsPanel({ config }: Props) {
  const [container, setContainer] = useState<string>('');
  const [destination, setDestination] = useState<string>('');
  
  if (!config) return null;
  
  const containers = config.config.warehouse.containers;
  const stations = config.config.warehouse.stations;
  const slots = config.config.warehouse.slots || [];
  
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!container || !destination) return;
    try {
      await api.taskSubmit({ container_id: container, destination });
    } catch (err) {
      console.error(err);
    }
  };

  const handleCancel = async () => {
    try {
      await api.taskCancel();
    } catch (err) {
      console.error(err);
    }
  };

  return (
    <section className="panel">
      <h2 className="panel-title">Warehouse Operations</h2>
      <form onSubmit={handleSubmit} style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
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
        <div style={{ display: 'flex', gap: '8px', marginTop: '8px' }}>
          <button type="submit" disabled={!container || !destination}>Start Task</button>
          <button type="button" onClick={handleCancel}>Cancel</button>
        </div>
      </form>
    </section>
  );
}
