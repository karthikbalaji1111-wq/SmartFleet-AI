import { useState } from 'react';
import type { DestinationConfig, NavigationTelemetry, PlanRequest, RouteModel } from '../api/types';
import { num } from './format';

const STATUS_TONE: Record<string, string> = {
  idle: 'idle',
  planning: 'warn',
  planned: 'info',
  navigating: 'warn',
  paused: 'warn',
  arrived: 'ok',
  cancelled: 'idle',
  failed: 'err',
};

interface Props {
  canDrive: boolean; // simulation running and physics online
  canPlan: boolean; // physics online
  destinations: DestinationConfig[];
  nav: NavigationTelemetry | undefined;
  route: RouteModel | null;
  error: string | null;
  pickMode: boolean;
  showGrid: boolean;
  onPickMode: (on: boolean) => void;
  onShowGrid: (on: boolean) => void;
  onPlan: (request: PlanRequest) => void;
  onAction: (action: 'start' | 'pause' | 'resume' | 'cancel') => void;
}

/**
 * Autonomous navigation controls. Planning, path following and progress all
 * happen on the backend; this panel only sends requests and displays the
 * navigation telemetry it streams back.
 */
export function NavigationPanel(props: Props) {
  const { canDrive, canPlan, destinations, nav, route, error, pickMode, showGrid } = props;
  const [selected, setSelected] = useState(destinations[0]?.id ?? '');
  const status = nav?.status ?? 'idle';
  const active = status === 'navigating' || status === 'paused';
  const metrics = route?.planner ?? nav?.planner ?? null;
  const interlock = nav?.interlock ?? [];

  return (
    <section className="panel">
      <h2 className="panel-title">
        Navigation <span className="tag">A* · static map</span>
        <span className={`badge badge--${STATUS_TONE[status] ?? 'idle'} badge--right`}>{status.toUpperCase()}</span>
      </h2>

      <div className="nav-destination">
        <select
          aria-label="Destination"
          value={selected || destinations[0]?.id}
          onChange={(e) => setSelected(e.target.value)}
          disabled={!canPlan || status === 'navigating'}
        >
          {destinations.map((d) => (
            <option key={d.id} value={d.id}>
              {d.label} ({num(d.x, 1)}, {num(d.y, 1)})
            </option>
          ))}
        </select>
        <button
          type="button"
          className="btn btn-sm btn-primary"
          disabled={!canPlan || status === 'navigating'}
          onClick={() => props.onPlan({ destination_id: selected || destinations[0].id })}
        >
          Plan route
        </button>
      </div>
      <div className="nav-row">
        <button
          type="button"
          className={`btn btn-sm ${pickMode ? 'is-active' : ''}`}
          aria-pressed={pickMode}
          disabled={!canPlan || status === 'navigating'}
          onClick={() => props.onPickMode(!pickMode)}
        >
          {pickMode ? 'Click the floor… (cancel)' : '⌖ Pick on floor'}
        </button>
        <label className="check">
          <input type="checkbox" checked={showGrid} onChange={(e) => props.onShowGrid(e.target.checked)} />
          Show occupancy grid
        </label>
      </div>

      <div className="button-row nav-actions">
        <button
          type="button"
          className="btn btn-primary"
          disabled={!canDrive || status !== 'planned' || interlock.length > 0}
          onClick={() => props.onAction('start')}
        >
          ▶ Start
        </button>
        {status === 'paused' ? (
          <button type="button" className="btn" disabled={!canDrive} onClick={() => props.onAction('resume')}>
            ▶ Resume
          </button>
        ) : (
          <button
            type="button"
            className="btn"
            disabled={!canPlan || status !== 'navigating'}
            onClick={() => props.onAction('pause')}
          >
            ❚❚ Pause
          </button>
        )}
        <button
          type="button"
          className="btn btn-stop"
          disabled={!canPlan || !(active || status === 'planned')}
          onClick={() => props.onAction('cancel')}
        >
          ✕ Cancel
        </button>
      </div>

      {nav?.destination && (
        <div className="nav-progress">
          <div className="nav-progress-head">
            <span>
              → <b>{nav.destination.label}</b>
              {nav.destination.snapped && (
                <small> (snapped {num(nav.destination.snap_distance, 2)} m to navigable floor)</small>
              )}
            </span>
            <span className="mono">{nav.progress === null ? '—' : `${Math.round(nav.progress * 100)}%`}</span>
          </div>
          <div className="mech-track" role="progressbar" aria-valuemin={0} aria-valuemax={1} aria-valuenow={nav.progress ?? 0}>
            <div className="mech-fill nav-fill" style={{ width: `${(nav.progress ?? 0) * 100}%` }} />
          </div>
          <dl className="kv kv--compact">
            <div>
              <dt>Waypoint</dt>
              <dd className="mono">
                {nav.waypoint_index ?? '—'} / {Math.max(0, nav.waypoint_count - 1)}
                {nav.phase ? ` · ${nav.phase}` : ''}
              </dd>
            </div>
            <div>
              <dt>Distance to goal / remaining route</dt>
              <dd className="mono">
                {num(nav.distance_to_goal, 2)} / {num(nav.remaining_distance, 2)} m
              </dd>
            </div>
            <div>
              <dt>Cross-track · heading error</dt>
              <dd className="mono">
                {num(nav.cross_track_error, 3)} m · {nav.heading_error === null ? '—' : `${num((nav.heading_error * 180) / Math.PI, 1)}°`}
              </dd>
            </div>
            <div>
              <dt>Autonomous command</dt>
              <dd className="mono">
                v {num(nav.command_linear, 2, true)} m/s · ω {num(nav.command_angular, 2, true)} rad/s
              </dd>
            </div>
            <div>
              <dt>Elapsed (sim time)</dt>
              <dd className="mono">{num(nav.elapsed, 1)} s</dd>
            </div>
          </dl>
        </div>
      )}

      {metrics && (
        <dl className="kv kv--compact nav-metrics">
          <div>
            <dt>Planner</dt>
            <dd className="mono">{metrics.status}</dd>
          </div>
          <div>
            <dt>Path length (A* cells)</dt>
            <dd className="mono">
              {num(metrics.length, 2)} m ({num(metrics.raw_length, 2)} m)
            </dd>
          </div>
          <div>
            <dt>Waypoints / raw points</dt>
            <dd className="mono">
              {metrics.waypoint_count} / {metrics.raw_point_count}
            </dd>
          </div>
          <div>
            <dt>Nodes expanded · planning time</dt>
            <dd className="mono">
              {metrics.expanded} · {num(metrics.time_ms, 1)} ms
            </dd>
          </div>
        </dl>
      )}

      {interlock.length > 0 && (
        <div className="notice notice--warn" role="status">
          <b>Travel interlock</b>: autonomous driving is blocked until
          <ul>
            {interlock.map((r) => (
              <li key={r}>{r}</li>
            ))}
          </ul>
        </div>
      )}
      {(error || nav?.reason) && (
        <p className={`notice ${status === 'failed' || error ? 'notice--err' : ''}`}>{error ?? nav?.reason}</p>
      )}
      <p className="hint">
        Routes are planned on the known static map with the robot's footprint and clearance. Manual drive pauses
        navigation; STOP / Space cancels it.
      </p>
    </section>
  );
}
