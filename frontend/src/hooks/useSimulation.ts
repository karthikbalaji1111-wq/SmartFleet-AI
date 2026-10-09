import { useCallback, useEffect, useRef, useState } from 'react';
import { ApiError, api, telemetryUrl } from '../api/client';
import type { ReadyResponse, SimulationSnapshot, TelemetryMessage, WarehouseConfigResponse } from '../api/types';

export type ConnectionState = 'connecting' | 'online' | 'offline';

export interface LogEntry {
  key: string;
  time: Date;
  level: 'info' | 'warning' | 'error';
  source: string;
  message: string;
}

const MAX_LOG = 250;
let clientLogSeq = 0;

/**
 * Owns the connection to the backend: loads the canonical warehouse config,
 * streams telemetry over the WebSocket (auto-reconnecting), and exposes the
 * simulation controls. The latest snapshot is also kept in a ref so the 3D
 * view can read it every animation frame without re-rendering React.
 */
export function useSimulation() {
  const [config, setConfig] = useState<WarehouseConfigResponse | null>(null);
  const [snapshot, setSnapshot] = useState<SimulationSnapshot | null>(null);
  const [connection, setConnection] = useState<ConnectionState>('connecting');
  const [readiness, setReadiness] = useState<ReadyResponse | null>(null);
  const [log, setLog] = useState<LogEntry[]>([]);
  const snapshotRef = useRef<SimulationSnapshot | null>(null);
  const seenServerEvents = useRef(new Set<string>());
  const lastClientError = useRef<string | null>(null);

  const addLog = useCallback((entry: Omit<LogEntry, 'key' | 'time'> & { key?: string; time?: Date }) => {
    setLog((prev) =>
      [
        { ...entry, key: entry.key ?? `c-${++clientLogSeq}`, time: entry.time ?? new Date() },
        ...prev,
      ].slice(0, MAX_LOG),
    );
  }, []);

  // ---- canonical warehouse configuration (retry until the backend is up) ----
  useEffect(() => {
    let cancelled = false;
    let timer: number | undefined;
    let warned = false;
    const load = async () => {
      try {
        const cfg = await api.warehouseConfig();
        if (cancelled) return;
        setConfig(cfg);
        addLog({
          level: 'info',
          source: 'client',
          message: `Loaded warehouse "${cfg.config.warehouse.name}": ${cfg.static_geometry.length} collision boxes, robot model "${cfg.robot_model.name}"`,
        });
        setReadiness(await api.ready().catch(() => null));
      } catch (err) {
        if (cancelled) return;
        if (!warned) {
          warned = true;
          addLog({
            level: 'error',
            source: 'client',
            message: `Cannot load warehouse config (${(err as Error).message}). Is the backend running on port 8000? Retrying…`,
          });
        }
        timer = window.setTimeout(load, 2000);
      }
    };
    void load();
    return () => {
      cancelled = true;
      window.clearTimeout(timer);
    };
  }, [addLog]);

  // ---- live telemetry over WebSocket ----
  useEffect(() => {
    let ws: WebSocket | null = null;
    let retry: number | undefined;
    let delay = 1000;
    let closedByUs = false;
    let everConnected = false;

    const connect = () => {
      setConnection((c) => (c === 'online' ? c : 'connecting'));
      ws = new WebSocket(telemetryUrl());
      ws.onopen = () => {
        delay = 1000;
        everConnected = true;
        setConnection('online');
        addLog({ level: 'info', source: 'client', message: 'Telemetry stream connected' });
      };
      ws.onmessage = (ev) => {
        const msg = JSON.parse(ev.data as string) as TelemetryMessage;
        if (msg.type !== 'telemetry') return;
        snapshotRef.current = msg.snapshot;
        setSnapshot(msg.snapshot);
        for (const e of msg.events) {
          // id + timestamp: survives reconnects and distinguishes backend restarts.
          const key = `s-${e.id}-${e.timestamp}`;
          if (seenServerEvents.current.has(key)) continue;
          seenServerEvents.current.add(key);
          addLog({ key, time: new Date(e.timestamp), level: e.level, source: e.source, message: e.message });
        }
      };
      ws.onclose = () => {
        setConnection('offline');
        snapshotRef.current = null;
        setSnapshot(null);
        if (closedByUs) return;
        if (everConnected) {
          everConnected = false;
          addLog({ level: 'warning', source: 'client', message: 'Telemetry stream lost; reconnecting…' });
        }
        retry = window.setTimeout(connect, delay);
        delay = Math.min(delay * 2, 5000);
      };
    };
    connect();
    return () => {
      closedByUs = true;
      window.clearTimeout(retry);
      ws?.close();
    };
  }, [addLog]);

  // Re-check readiness when the physics state flips (e.g. backend restarted).
  const ready = snapshot?.ready;
  useEffect(() => {
    if (ready === undefined) return;
    api.ready().then(setReadiness).catch(() => undefined);
  }, [ready]);

  // ---- controls ----
  const control = useCallback(
    async (label: string, fn: () => Promise<{ message: string }>) => {
      try {
        const res = await fn();
        addLog({ level: 'info', source: 'client', message: `${label}: ${res.message}` });
      } catch (err) {
        addLog({ level: 'error', source: 'client', message: `${label} failed: ${(err as Error).message}` });
      }
    },
    [addLog],
  );

  const start = useCallback(() => control('Start', api.start), [control]);
  const pause = useCallback(() => control('Pause', api.pause), [control]);
  const reset = useCallback(() => control('Reset', api.reset), [control]);

  const drive = useCallback(
    async (linear: number, angular: number, duration: number) => {
      try {
        await api.velocity(linear, angular, duration);
        lastClientError.current = null;
      } catch (err) {
        const message = err instanceof ApiError ? err.message : String(err);
        if (message !== lastClientError.current) {
          lastClientError.current = message;
          addLog({ level: 'error', source: 'client', message: `Drive command rejected: ${message}` });
        }
      }
    },
    [addLog],
  );

  const stop = useCallback(async () => {
    try {
      await api.stop();
    } catch (err) {
      addLog({ level: 'error', source: 'client', message: `Stop failed: ${(err as Error).message}` });
    }
  }, [addLog]);

  return { config, snapshot, snapshotRef, connection, readiness, log, start, pause, reset, drive, stop };
}
