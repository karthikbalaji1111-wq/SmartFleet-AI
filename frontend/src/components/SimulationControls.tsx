import type { ReadyResponse, SimulationSnapshot } from '../api/types';

interface Props {
  snapshot: SimulationSnapshot | null;
  readiness: ReadyResponse | null;
  onStart: () => void;
  onPause: () => void;
  onReset: () => void;
}

export function SimulationControls({ snapshot, readiness, onStart, onPause, onReset }: Props) {
  const ready = snapshot?.ready ?? false;
  const status = snapshot?.status;
  return (
    <section className="panel">
      <h2 className="panel-title">Simulation</h2>
      <div className="button-row">
        <button type="button" className="btn btn-primary" onClick={onStart} disabled={!ready || status === 'running'}>
          ▶ {status === 'paused' ? 'Resume' : 'Start'}
        </button>
        <button type="button" className="btn" onClick={onPause} disabled={!ready || status !== 'running'}>
          ❚❚ Pause
        </button>
        <button type="button" className="btn" onClick={onReset} disabled={!ready}>
          ↺ Reset
        </button>
      </div>
      <dl className="kv kv--compact">
        <div>
          <dt>Physics</dt>
          <dd>
            PyBullet DIRECT · {snapshot ? `${snapshot.physics_hz} Hz` : '—'}
          </dd>
        </div>
        <div>
          <dt>Step</dt>
          <dd className="mono">{snapshot?.world?.step ?? '—'}</dd>
        </div>
      </dl>
      {snapshot && !ready && (
        <p className="notice notice--err">
          Physics unavailable{readiness?.detail ? `: ${readiness.detail}` : ''}. Start the backend from the
          <code>smartfleet</code> conda environment.
        </p>
      )}
      {status === 'stopped' && ready && (
        <p className="notice">Stopped (safe state): physics is frozen and no motion is requested.</p>
      )}
    </section>
  );
}
