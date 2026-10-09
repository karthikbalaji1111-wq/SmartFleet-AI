import type { ReadyResponse, SimulationSnapshot } from '../api/types';
import type { ConnectionState } from '../hooks/useSimulation';
import { clock } from './format';

interface Props {
  connection: ConnectionState;
  snapshot: SimulationSnapshot | null;
  readiness: ReadyResponse | null;
}

export function StatusBar({ connection, snapshot, readiness }: Props) {
  const physicsReady = snapshot?.ready ?? readiness?.ready ?? false;
  const status = snapshot?.status;
  return (
    <header className="topbar">
      <div className="brand">
        <svg viewBox="0 0 32 32" width="28" height="28" aria-hidden="true">
          <rect width="32" height="32" rx="6" fill="#1b2330" />
          <rect x="7" y="15" width="18" height="8" rx="1.5" fill="#f5b324" />
          <rect x="9" y="6" width="3" height="10" fill="#8b96a5" />
          <circle cx="11" cy="24" r="2.5" fill="#d8dee9" />
          <circle cx="21" cy="24" r="2.5" fill="#d8dee9" />
        </svg>
        <div>
          <div className="brand-name">SmartFleet AI</div>
          <div className="brand-sub">Robot Lab · Milestone 1</div>
        </div>
      </div>
      <div className="status-pills" role="status" aria-live="polite">
        <span className={`pill pill--${connection === 'online' ? 'ok' : connection === 'connecting' ? 'warn' : 'err'}`}>
          <i />
          Backend {connection}
        </span>
        <span className={`pill pill--${physicsReady ? 'ok' : 'err'}`} title={readiness?.detail ?? undefined}>
          <i />
          Physics {physicsReady ? 'ready' : 'unavailable'}
        </span>
        <span className={`pill pill--${status === 'running' ? 'ok' : status === 'paused' ? 'warn' : 'idle'}`}>
          <i />
          {status ? status.toUpperCase() : 'NO DATA'}
        </span>
        <span className="pill pill--mono">t = {clock(snapshot?.world?.sim_time)}</span>
      </div>
    </header>
  );
}
