import { PerceptionResult } from '../api/types';

interface PerceptionPanelProps {
  perception: PerceptionResult | null;
}

export function PerceptionPanel({ perception }: PerceptionPanelProps) {
  if (!perception) {
    return (
      <div className="panel perception-panel">
        <header>
          <h2>Visual Perception</h2>
        </header>
        <div className="panel-content empty-state">Waiting for perception feed...</div>
      </div>
    );
  }

  return (
    <div className="panel perception-panel">
      <header>
        <h2>Visual Perception</h2>
        <div className="badges">
          <span className="badge badge-info">{perception.latency_ms.toFixed(1)} ms</span>
        </div>
      </header>

      <div className="panel-content">
        <div className="camera-feed" style={{ marginBottom: '1rem', textAlign: 'center' }}>
          <img
            src={`data:image/jpeg;base64,${perception.rgb_base64}`}
            alt="Live Camera Feed"
            style={{ maxWidth: '100%', borderRadius: '4px', border: '1px solid #333' }}
          />
        </div>

        <div className="table-container">
          <table className="data-table">
            <thead>
              <tr>
                <th>Object</th>
                <th>Distance (m)</th>
                <th>XYZ (m)</th>
                <th>AABB Boundary Distance (cm)</th>
              </tr>
            </thead>
            <tbody>
              {perception.objects.map((obj, i) => (
                <tr key={i}>
                  <td>
                    <strong>{obj.category}</strong>
                    <br />
                    <span className="dim">{obj.instance_id}</span>
                  </td>
                  <td>{obj.distance.toFixed(2)}</td>
                  <td>
                    {obj.position.x.toFixed(2)}, {obj.position.y.toFixed(2)}, {obj.position.z.toFixed(2)}
                  </td>
                  <td>
                    {obj.aabb_boundary_distance !== null ? (
                      <span className={obj.aabb_boundary_distance < 50 ? 'ok' : 'warn'}>
                        {obj.aabb_boundary_distance.toFixed(1)}
                      </span>
                    ) : (
                      'N/A'
                    )}
                  </td>
                </tr>
              ))}
              {perception.objects.length === 0 && (
                <tr>
                  <td colSpan={4} className="empty-state">No objects detected.</td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
}
