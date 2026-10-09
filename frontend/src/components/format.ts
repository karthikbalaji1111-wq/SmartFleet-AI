export function num(value: number | undefined | null, digits = 2, signed = false): string {
  if (value === undefined || value === null || !Number.isFinite(value)) return '—';
  const fixed = value.toFixed(digits);
  // Avoid "-0.00".
  const clean = Number(fixed) === 0 ? (0).toFixed(digits) : fixed;
  return signed && Number(clean) > 0 ? `+${clean}` : clean;
}

export function clock(seconds: number | undefined): string {
  if (seconds === undefined || !Number.isFinite(seconds)) return '—';
  const m = Math.floor(seconds / 60);
  const s = seconds - m * 60;
  return `${String(m).padStart(2, '0')}:${s.toFixed(2).padStart(5, '0')}`;
}
