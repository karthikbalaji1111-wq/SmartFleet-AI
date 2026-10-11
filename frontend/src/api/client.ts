import type {
  ControlResponse,
  MechanismCommandResponse,
  MechanismName,
  NavigationResponse,
  OccupancyGridData,
  PlanRequest,
  ReadyResponse,
  WarehouseConfigResponse,
} from './types';

export class ApiError extends Error {
  constructor(
    message: string,
    readonly status: number,
  ) {
    super(message);
  }
}

function describeDetail(detail: unknown): string {
  if (typeof detail === 'string') return detail;
  if (Array.isArray(detail)) {
    return detail
      .map((d: { loc?: unknown[]; msg?: string }) => `${(d.loc ?? []).slice(1).join('.') || 'body'}: ${d.msg}`)
      .join('; ');
  }
  return JSON.stringify(detail);
}

const API_BASE = (import.meta.env.VITE_BACKEND_URL as string | undefined) ?? '';

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  let res: Response;
  try {
    res = await fetch(`${API_BASE}${path}`, { ...init, headers: { 'content-type': 'application/json', ...init?.headers } });
  } catch {
    throw new ApiError('Backend unreachable', 0);
  }
  const body = await res.json().catch(() => null);
  if (!res.ok) {
    const detail = body && typeof body === 'object' && 'detail' in body ? describeDetail(body.detail) : res.statusText;
    throw new ApiError(detail || `HTTP ${res.status}`, res.status);
  }
  return body as T;
}

export const api = {
  warehouseConfig: () => request<WarehouseConfigResponse>('/api/config/warehouse'),
  ready: async (): Promise<ReadyResponse> => {
    // 503 still carries a ReadyResponse body describing why.
    const res = await fetch(`${API_BASE}/api/ready`);
    return (await res.json()) as ReadyResponse;
  },
  start: () => request<ControlResponse>('/api/simulation/start', { method: 'POST' }),
  pause: () => request<ControlResponse>('/api/simulation/pause', { method: 'POST' }),
  reset: () => request<ControlResponse>('/api/simulation/reset', { method: 'POST' }),
  stop: () => request<ControlResponse>('/api/robot/stop', { method: 'POST' }),
  estop: () => request<ControlResponse>('/api/robot/estop', { method: 'POST' }),
  velocity: (linear: number, angular: number, duration: number) =>
    request<unknown>('/api/robot/velocity', {
      method: 'POST',
      body: JSON.stringify({ linear, angular, duration }),
    }),
  mechanismTarget: (name: MechanismName, position: number) =>
    request<MechanismCommandResponse>(`/api/robot/${name}/target`, {
      method: 'POST',
      body: JSON.stringify({ position }),
    }),
  mechanismJog: (name: MechanismName, delta: number) =>
    request<MechanismCommandResponse>(`/api/robot/${name}/jog`, {
      method: 'POST',
      body: JSON.stringify({ delta }),
    }),
  mechanismStop: (name?: MechanismName) =>
    request<{ message: string }>(name ? `/api/robot/${name}/stop` : '/api/robot/mechanisms/stop', {
      method: 'POST',
    }),
  navigationGrid: () => request<OccupancyGridData>('/api/navigation/grid'),
  navigationPlan: (body: PlanRequest) =>
    request<NavigationResponse>('/api/navigation/plan', { method: 'POST', body: JSON.stringify(body) }),
  navigationAction: (action: 'start' | 'pause' | 'resume' | 'cancel') =>
    request<NavigationResponse>(`/api/navigation/${action}`, { method: 'POST' }),
  taskSubmit: (body: { container_id: string; destination: string; type?: 'storage' | 'retrieval'; priority?: number }) =>
    request<{ message: string; task_id: string }>('/api/tasks/submit', { method: 'POST', body: JSON.stringify(body) }),
  taskCancel: () =>
    request<{ message: string }>('/api/tasks/cancel', { method: 'POST' }),
  taskCancelSpecific: (taskId: string) =>
    request<{ message: string }>(`/api/tasks/${taskId}/cancel`, { method: 'POST' }),
  taskSetPriority: (taskId: string, priority: number) =>
    request<{ message: string }>(`/api/tasks/${taskId}/priority`, { method: 'POST', body: JSON.stringify({ priority }) }),

};

export function telemetryUrl(): string {
  const configuredWs = import.meta.env.VITE_WS_URL as string | undefined;
  if (configuredWs) {
    return `${configuredWs.replace(/\/$/, '')}/ws/telemetry`;
  }
  const scheme = window.location.protocol === 'https:' ? 'wss' : 'ws';
  return `${scheme}://${window.location.host}/ws/telemetry`;
}
