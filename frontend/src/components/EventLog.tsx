import { memo, useState } from 'react';
import type { LogEntry } from '../hooks/useSimulation';

const timeFmt = new Intl.DateTimeFormat(undefined, {
  hour: '2-digit',
  minute: '2-digit',
  second: '2-digit',
  hourCycle: 'h23',
});

export const EventLog = memo(function EventLog({ entries }: { entries: LogEntry[] }) {
  const [errorsOnly, setErrorsOnly] = useState(false);
  const shown = errorsOnly ? entries.filter((e) => e.level !== 'info') : entries;
  return (
    <section className="panel log-panel">
      <div className="log-head">
        <h2 className="panel-title">Event log</h2>
        <label className="check">
          <input type="checkbox" checked={errorsOnly} onChange={(e) => setErrorsOnly(e.target.checked)} />
          Warnings &amp; errors only
        </label>
      </div>
      <ol className="log" aria-live="polite">
        {shown.length === 0 && <li className="log-empty">No events yet.</li>}
        {shown.map((e) => (
          <li key={e.key} className={`log-row log-row--${e.level}`}>
            <time className="mono">{timeFmt.format(e.time)}</time>
            <span className="log-level">{e.level}</span>
            <span className="log-source">{e.source}</span>
            <span className="log-msg">{e.message}</span>
          </li>
        ))}
      </ol>
    </section>
  );
});
