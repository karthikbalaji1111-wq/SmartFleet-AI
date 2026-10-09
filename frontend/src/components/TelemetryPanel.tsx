import type { ActuatorTelemetry, WorldState } from '../api/types';
import { num } from './format';

function Gauge({ label, act, unit = 'm' }: { label: string; act: ActuatorTelemetry | undefined; unit?: string }) {
  const span = act ? act.upper - act.lower : 1;
  const pct = act ? Math.max(0, Math.min(100, ((act.position - act.lower) / span) * 100)) : 0;
  return (
    <div className="gauge">
      <div className="gauge-head">
        <span>{label}</span>
        <span className="mono">
          {num(act?.position, 3)} {unit}
          <span className="tag tag--muted">{act ? act.mode.toUpperCase() : '—'}</span>
        </span>
      </div>
      <div className="gauge-track" role="meter" aria-valuemin={act?.lower} aria-valuemax={act?.upper} aria-valuenow={act?.position}>
        <div className="gauge-fill" style={{ width: `${pct}%` }} />
      </div>
      <div className="gauge-scale mono">
        <span>{num(act?.lower, 2)}</span>
        <span>limit {num(act?.upper, 2)} {unit}</span>
      </div>
    </div>
  );
}

export function TelemetryPanel({ world, robotId }: { world: WorldState | null | undefined; robotId?: string }) {
  const r = world?.robot;
  const c = world?.container;
  return (
    <section className="panel">
      <h2 className="panel-title">
        Robot telemetry <span className="tag">{robotId ?? 'robot'} · PyBullet</span>
      </h2>
      <div className="metrics">
        <div className="metric">
          <span>X</span>
          <b className="mono">{num(r?.pose.position[0], 3)}</b>
          <small>m</small>
        </div>
        <div className="metric">
          <span>Y</span>
          <b className="mono">{num(r?.pose.position[1], 3)}</b>
          <small>m</small>
        </div>
        <div className="metric">
          <span>Heading</span>
          <b className="mono">{num(r?.pose.heading_deg, 1)}</b>
          <small>° ({num(r?.pose.heading, 3)} rad)</small>
        </div>
        <div className="metric">
          <span>Linear vel.</span>
          <b className="mono">{num(r?.velocity.forward, 3, true)}</b>
          <small>m/s · cmd {num(r?.command.linear, 2, true)}</small>
        </div>
        <div className="metric">
          <span>Angular vel.</span>
          <b className="mono">{num(r?.velocity.yaw_rate, 3, true)}</b>
          <small>rad/s · cmd {num(r?.command.angular, 2, true)}</small>
        </div>
        <div className="metric">
          <span>Wheels L / R</span>
          <b className="mono">
            {num(r?.wheels.left.velocity, 1, true)} / {num(r?.wheels.right.velocity, 1, true)}
          </b>
          <small>rad/s measured{r?.wheels.brake_engaged ? ' · parking brake' : ''}</small>
        </div>
      </div>
      <Gauge label="Lift height" act={r?.lift} />
      <Gauge label="Fork extension" act={r?.forks} />
      <p className="hint">
        Lift and forks are physical prismatic joints held at their lower limits. Commanded lift/fork motion is
        deferred to a later milestone.
      </p>
      <dl className="kv kv--compact">
        <div>
          <dt>Tilt (roll / pitch)</dt>
          <dd className="mono">
            {num(r?.pose.roll, 3, true)} / {num(r?.pose.pitch, 3, true)} rad
          </dd>
        </div>
        <div>
          <dt>Container {c?.id ?? ''}</dt>
          <dd className="mono">
            {c ? `${num(c.position[0], 2)}, ${num(c.position[1], 2)}, ${num(c.position[2], 2)} m` : '—'}
          </dd>
        </div>
      </dl>
    </section>
  );
}
