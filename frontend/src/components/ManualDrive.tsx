import { useCallback, useEffect, useRef, useState, type PointerEvent as ReactPointerEvent } from 'react';
import type { MotionLimits, RobotState } from '../api/types';
import { num } from './format';

type Dir = 'forward' | 'back' | 'left' | 'right';

const KEY_DIRS: Record<string, Dir> = {
  KeyW: 'forward',
  ArrowUp: 'forward',
  KeyS: 'back',
  ArrowDown: 'back',
  KeyA: 'left',
  ArrowLeft: 'left',
  KeyD: 'right',
  ArrowRight: 'right',
};

const RESEND_MS = 120; // well inside the dead-man duration below
const COMMAND_DURATION_S = 0.4;

interface Props {
  enabled: boolean;
  limits: MotionLimits | undefined;
  robot: RobotState | undefined;
  onDrive: (linear: number, angular: number, duration: number) => void;
  onStop: () => void;
}

/**
 * Temporary verification controls. Hold a button (or W/A/S/D / arrow keys) to
 * drive; requests are re-sent while held and carry a short dead-man timeout,
 * so releasing — or losing the connection — stops the robot.
 */
export function ManualDrive({ enabled, limits, robot, onDrive, onStop }: Props) {
  const maxV = limits?.max_linear_velocity ?? 1;
  const maxW = limits?.max_angular_velocity ?? 1.5;
  const [speed, setSpeed] = useState(0.5);
  const [turn, setTurn] = useState(0.8);
  const [held, setHeld] = useState<Set<Dir>>(new Set());

  const keysRef = useRef(new Set<Dir>());
  const pointerRef = useRef<Dir | null>(null);
  const movingRef = useRef(false);
  const params = useRef({ speed, turn, enabled, onDrive, onStop });
  params.current = { speed, turn, enabled, onDrive, onStop };

  const pump = useCallback(() => {
    const { speed: v, turn: w, enabled: on, onDrive: drive, onStop: stop } = params.current;
    const dirs = new Set(keysRef.current);
    if (pointerRef.current) dirs.add(pointerRef.current);
    const linear = on ? ((dirs.has('forward') ? 1 : 0) - (dirs.has('back') ? 1 : 0)) * v : 0;
    const angular = on ? ((dirs.has('left') ? 1 : 0) - (dirs.has('right') ? 1 : 0)) * w : 0;
    if (linear !== 0 || angular !== 0) {
      movingRef.current = true;
      drive(Number(linear.toFixed(3)), Number(angular.toFixed(3)), COMMAND_DURATION_S);
    } else if (movingRef.current) {
      movingRef.current = false;
      stop();
    }
  }, []);

  const refreshHeld = useCallback(() => {
    const dirs = new Set(keysRef.current);
    if (pointerRef.current) dirs.add(pointerRef.current);
    setHeld(dirs);
  }, []);

  // Periodic re-send while any direction is held.
  useEffect(() => {
    const id = window.setInterval(() => {
      if (keysRef.current.size || pointerRef.current) pump();
    }, RESEND_MS);
    return () => window.clearInterval(id);
  }, [pump]);

  // Releasing everything when drive becomes unavailable.
  useEffect(() => {
    if (!enabled) {
      keysRef.current.clear();
      pointerRef.current = null;
      movingRef.current = false;
      setHeld(new Set());
    }
  }, [enabled]);

  // Keyboard driving (ignored while typing in a form field).
  useEffect(() => {
    const isFormField = (t: EventTarget | null) =>
      t instanceof HTMLElement && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable);
    const down = (e: KeyboardEvent) => {
      if (e.code === 'Space' && !isFormField(e.target)) {
        e.preventDefault();
        keysRef.current.clear();
        pointerRef.current = null;
        movingRef.current = false;
        refreshHeld();
        params.current.onStop();
        return;
      }
      const dir = KEY_DIRS[e.code];
      if (!dir || isFormField(e.target) || e.repeat) return;
      e.preventDefault();
      keysRef.current.add(dir);
      refreshHeld();
      pump();
    };
    const up = (e: KeyboardEvent) => {
      const dir = KEY_DIRS[e.code];
      if (!dir) return;
      keysRef.current.delete(dir);
      refreshHeld();
      pump();
    };
    const blur = () => {
      keysRef.current.clear();
      pointerRef.current = null;
      refreshHeld();
      pump();
    };
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    window.addEventListener('blur', blur);
    return () => {
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('blur', blur);
    };
  }, [pump, refreshHeld]);

  const pad = (dir: Dir, label: string, glyph: string) => ({
    className: `pad-btn pad-${dir} ${held.has(dir) ? 'is-held' : ''}`,
    disabled: !enabled,
    'aria-label': label,
    title: label,
    children: glyph,
    onPointerDown: (e: ReactPointerEvent<HTMLButtonElement>) => {
      e.currentTarget.setPointerCapture(e.pointerId);
      pointerRef.current = dir;
      refreshHeld();
      pump();
    },
    onPointerUp: () => {
      pointerRef.current = null;
      refreshHeld();
      pump();
    },
    onPointerCancel: () => {
      pointerRef.current = null;
      refreshHeld();
      pump();
    },
  });

  const cmd = robot?.command;
  return (
    <section className="panel">
      <h2 className="panel-title">
        Manual drive <span className="tag">verification only</span>
      </h2>
      <div className="drive">
        <div className="pad" role="group" aria-label="Drive pad">
          <button type="button" {...pad('forward', 'Drive forward (W / ↑)', '▲')} />
          <button type="button" {...pad('left', 'Rotate left (A / ←)', '⟲')} />
          <button
            type="button"
            className="pad-btn pad-stop"
            disabled={!enabled}
            onClick={() => {
              keysRef.current.clear();
              pointerRef.current = null;
              movingRef.current = false;
              refreshHeld();
              onStop();
            }}
            title="Stop (Space)"
          >
            STOP
          </button>
          <button type="button" {...pad('right', 'Rotate right (D / →)', '⟳')} />
          <button type="button" {...pad('back', 'Reverse (S / ↓)', '▼')} />
        </div>
        <div className="sliders">
          <label>
            <span>
              Speed <b className="mono">{num(speed, 2)} m/s</b>
            </span>
            <input
              type="range"
              min={0.1}
              max={maxV}
              step={0.05}
              value={Math.min(speed, maxV)}
              onChange={(e) => setSpeed(Number(e.target.value))}
            />
          </label>
          <label>
            <span>
              Turn rate <b className="mono">{num(turn, 2)} rad/s</b>
            </span>
            <input
              type="range"
              min={0.1}
              max={maxW}
              step={0.05}
              value={Math.min(turn, maxW)}
              onChange={(e) => setTurn(Number(e.target.value))}
            />
          </label>
          <div className="cmd-readout mono">
            cmd v {num(cmd?.target_linear, 2, true)} m/s · ω {num(cmd?.target_angular, 2, true)} rad/s
            <span className={`dot ${cmd?.active ? 'dot--on' : ''}`} title="Request active" />
          </div>
        </div>
      </div>
      <p className="hint">
        {enabled
          ? 'Hold a button or W/A/S/D (arrows). Release to stop; Space = stop. Limits: ' +
            `±${maxV} m/s, ±${maxW} rad/s.`
          : 'Start the simulation to enable manual drive.'}
      </p>
    </section>
  );
}
