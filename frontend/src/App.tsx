import { EventLog } from './components/EventLog';
import { ManualDrive } from './components/ManualDrive';
import { SimulationControls } from './components/SimulationControls';
import { StatusBar } from './components/StatusBar';
import { TelemetryPanel } from './components/TelemetryPanel';
import { useSimulation } from './hooks/useSimulation';
import { WarehouseViewport } from './scene/WarehouseViewport';

export default function App() {
  const sim = useSimulation();
  const world = sim.snapshot?.world;
  const driveEnabled = sim.connection === 'online' && sim.snapshot?.ready === true && sim.snapshot.status === 'running';

  return (
    <div className="app">
      <StatusBar connection={sim.connection} snapshot={sim.snapshot} readiness={sim.readiness} />
      <main className="layout">
        <div className="stage">
          <WarehouseViewport data={sim.config} snapshotRef={sim.snapshotRef} hasTelemetry={Boolean(world)} />
          <EventLog entries={sim.log} />
        </div>
        <aside className="sidebar">
          <SimulationControls
            snapshot={sim.snapshot}
            readiness={sim.readiness}
            onStart={sim.start}
            onPause={sim.pause}
            onReset={sim.reset}
          />
          <ManualDrive
            enabled={driveEnabled}
            limits={sim.config?.config.robot.limits}
            robot={world?.robot}
            onDrive={sim.drive}
            onStop={sim.stop}
          />
          <TelemetryPanel world={world} robotId={sim.config?.config.robot.id} />
        </aside>
      </main>
    </div>
  );
}
