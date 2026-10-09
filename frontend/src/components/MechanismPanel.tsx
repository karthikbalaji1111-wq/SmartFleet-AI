import { useState, type ReactNode } from 'react';
import type { ActuatorConfig, LiftPreset, MechanismName, MechanismTelemetry } from '../api/types';
import { num } from './format';

const JOG_STEP = 0.05; // m per jog click (backend accepts up to max_jog_step)

const WORDS: Record<MechanismName, { title: string; up: string; down: string; upShort: string; downShort: string }> = {
  lift: { title: 'Lift', up: 'Raising', down: 'Lowering', upShort: 'Raise', downShort: 'Lower' },
  forks: { title: 'Forks', up: 'Extending', down: 'Retracting', upShort: 'Extend', downShort: 'Retract' },
};

const FAULT_TEXT: Record<NonNullable<MechanismTelemetry['fault']>, string> = {
  overload: 'Stopped: the motor hit its force limit (obstruction or overload).',
  stalled: 'Stopped: no motion towards the target (obstruction).',
  tilt: 'Stopped: the chassis tilted beyond the safe limit.',
};

function stateLabel(name: MechanismName, m: MechanismTelemetry | undefined): { text: string; tone: string } {
  if (!m) return { text: '—', tone: 'idle' };
  if (m.state === 'blocked') return { text: 'BLOCKED', tone: 'err' };
  if (m.state === 'moving') return { text: (m.error > 0 ? WORDS[name].up : WORDS[name].down).toUpperCase(), tone: 'warn' };
  return { text: 'HOLDING', tone: 'ok' };
}

interface CardProps {
  name: MechanismName;
  mech: MechanismTelemetry | undefined;
  cfg: ActuatorConfig | undefined;
  enabled: boolean;
  canStop: boolean;
  onTarget: (name: MechanismName, position: number) => void;
  onJog: (name: MechanismName, delta: number) => void;
  onStop: (name: MechanismName) => void;
  children?: ReactNode;
}

function MechanismCard({ name, mech, cfg, enabled, canStop, onTarget, onJog, onStop, children }: CardProps) {
  const [pending, setPending] = useState<number | null>(null);
  const lower = cfg?.lower ?? 0;
  const upper = cfg?.upper ?? 1;
  const span = upper - lower || 1;
  const pct = (v: number | undefined) => (v === undefined ? 0 : Math.max(0, Math.min(100, ((v - lower) / span) * 100)));
  const sliderValue = pending ?? mech?.target ?? lower;
  const status = stateLabel(name, mech);
  const words = WORDS[name];

  return (
    <div className={`mech mech--${status.tone}`}>
      <div className="mech-head">
        <span className="mech-title">{words.title}</span>
        <span className={`badge badge--${status.tone}`}>{status.text}</span>
      </div>

      <div className="mech-readout">
        <div>
          <b className="mono">{num(mech?.position, 3)}</b>
          <small>m measured</small>
        </div>
        <div>
          <span className="mono">{num(mech?.target, 3)} m</span>
          <small>target</small>
        </div>
        <div>
          <span className="mono">{num(mech?.velocity, 2, true)} m/s</span>
          <small>velocity</small>
        </div>
      </div>

      <div
        className="mech-track"
        role="meter"
        aria-label={`${words.title} position`}
        aria-valuemin={lower}
        aria-valuemax={upper}
        aria-valuenow={mech?.position}
      >
        <div className="mech-fill" style={{ width: `${pct(mech?.position)}%` }} />
        <div className="mech-target" style={{ left: `${pct(mech?.target)}%` }} title="Target" />
        {pending !== null && <div className="mech-pending" style={{ left: `${pct(pending)}%` }} title="Pending" />}
      </div>
      <div className="gauge-scale mono">
        <span>{num(lower, 2)}</span>
        <span>limit {num(upper, 2)} m</span>
      </div>

      <div className="mech-slider">
        <input
          type="range"
          min={lower}
          max={upper}
          step={0.01}
          value={sliderValue}
          disabled={!enabled}
          aria-label={`${words.title} target`}
          onChange={(e) => setPending(Number(e.target.value))}
        />
        <button
          type="button"
          className="btn btn-primary btn-sm"
          disabled={!enabled || pending === null}
          aria-label={`Move ${words.title.toLowerCase()} to ${num(sliderValue, 2)} m`}
          onClick={() => {
            if (pending === null) return;
            onTarget(name, pending);
            setPending(null);
          }}
        >
          Move to {num(sliderValue, 2)} m
        </button>
      </div>

      <div className="mech-buttons">
        <button type="button" className="btn btn-sm" disabled={!enabled} onClick={() => onTarget(name, lower)}>
          {words.downShort} fully
        </button>
        <button
          type="button"
          className="btn btn-sm"
          disabled={!enabled}
          aria-label={`${words.downShort} ${words.title.toLowerCase()} ${JOG_STEP * 100} cm`}
          onClick={() => onJog(name, -JOG_STEP)}
        >
          − {JOG_STEP * 100} cm
        </button>
        <button
          type="button"
          className="btn btn-sm btn-stop"
          disabled={!canStop}
          aria-label={`Stop ${words.title.toLowerCase()}`}
          onClick={() => onStop(name)}
        >
          ■ Stop
        </button>
        <button
          type="button"
          className="btn btn-sm"
          disabled={!enabled}
          aria-label={`${words.upShort} ${words.title.toLowerCase()} ${JOG_STEP * 100} cm`}
          onClick={() => onJog(name, JOG_STEP)}
        >
          + {JOG_STEP * 100} cm
        </button>
        <button type="button" className="btn btn-sm" disabled={!enabled} onClick={() => onTarget(name, upper)}>
          {words.upShort} fully
        </button>
      </div>

      {children}

      {mech?.state === 'blocked' && (
        <p className="notice notice--err">
          {FAULT_TEXT[mech.fault ?? 'stalled']} Holding at {num(mech.position, 3)} m; see the event log for what it
          touched. Send a new target to retry.
        </p>
      )}
    </div>
  );
}

interface Props {
  enabled: boolean;
  canStop: boolean;
  lift: MechanismTelemetry | undefined;
  forks: MechanismTelemetry | undefined;
  forkSurfaceHeight: number | undefined;
  liftConfig: ActuatorConfig | undefined;
  forkConfig: ActuatorConfig | undefined;
  presets: LiftPreset[];
  onTarget: (name: MechanismName, position: number) => void;
  onJog: (name: MechanismName, delta: number) => void;
  onStop: (name?: MechanismName) => void;
}

/**
 * Lift and fork controls. Every button sends a command to the backend; the
 * values shown are the joint states PyBullet reports back over telemetry.
 */
export function MechanismPanel(props: Props) {
  const { enabled, canStop, lift, forks, forkSurfaceHeight, presets, onTarget, onJog, onStop } = props;
  return (
    <section className="panel">
      <h2 className="panel-title">
        Lift &amp; forks <span className="tag">PyBullet joints</span>
      </h2>
      <MechanismCard
        name="lift"
        mech={lift}
        cfg={props.liftConfig}
        enabled={enabled}
        canStop={canStop}
        onTarget={onTarget}
        onJog={onJog}
        onStop={onStop}
      >
        <div className="mech-presets" role="group" aria-label="Lift presets">
          {presets.map((p) => (
            <button
              key={p.id}
              type="button"
              className={`chip ${lift && Math.abs(lift.target - p.position) < 1e-3 ? 'is-active' : ''}`}
              disabled={!enabled}
              aria-label={`${p.label}: fork surface ${p.surface_height.toFixed(2)} m above floor`}
              title={`Fork surface ${p.surface_height.toFixed(2)} m above floor`}
              onClick={() => onTarget('lift', p.position)}
            >
              {p.label}
              <small className="mono">{p.surface_height.toFixed(2)}</small>
            </button>
          ))}
        </div>
        <dl className="kv kv--compact">
          <div>
            <dt>Fork surface above floor</dt>
            <dd className="mono">{num(forkSurfaceHeight, 3)} m</dd>
          </div>
          <div>
            <dt>Lift motor force</dt>
            <dd className="mono">{num(lift?.applied_force, 0)} N</dd>
          </div>
        </dl>
      </MechanismCard>
      <MechanismCard
        name="forks"
        mech={forks}
        cfg={props.forkConfig}
        enabled={enabled}
        canStop={canStop}
        onTarget={onTarget}
        onJog={onJog}
        onStop={onStop}
      />
      <button type="button" className="btn btn-stop btn-block" disabled={!canStop} onClick={() => onStop()}>
        ■ Stop lift &amp; forks
      </button>
      <p className="hint">
        {enabled
          ? 'Targets outside the joint limits are rejected by the backend. Motion runs at the configured motor speed limits.'
          : 'Start the simulation to move the lift and forks.'}
      </p>
    </section>
  );
}
