import { EventLog } from './components/EventLog';
import { ManualDrive } from './components/ManualDrive';
import { MechanismPanel } from './components/MechanismPanel';
import { SimulationControls } from './components/SimulationControls';
import { StatusBar } from './components/StatusBar';
import { TelemetryPanel } from './components/TelemetryPanel';
import { useSimulation } from './hooks/useSimulation';
import { WarehouseViewport } from './scene/WarehouseViewport';

export default function App() {
  const sim = useSimulation();
  const world = sim.snapshot?.world;
  const physicsOnline = sim.connection === 'online' && sim.snapshot?.ready === true;
  const driveEnabled = physicsOnline && sim.snapshot?.status === 'running';

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
            canEstop={physicsOnline}
            limits={sim.config?.config.robot.limits}
            robot={world?.robot}
            onDrive={sim.drive}
            onStop={sim.stop}
            onEstop={sim.estop}
          />
          <MechanismPanel
            enabled={driveEnabled}
            canStop={physicsOnline}
            lift={world?.robot.lift}
            forks={world?.robot.forks}
            forkSurfaceHeight={world?.robot.fork_surface_height}
            liftConfig={sim.config?.config.robot.lift}
            forkConfig={sim.config?.config.robot.forks}
            presets={sim.config?.lift_presets ?? []}
            onTarget={sim.setMechanismTarget}
            onJog={sim.jogMechanism}
            onStop={sim.stopMechanism}
          />
          <TelemetryPanel world={world} robotId={sim.config?.config.robot.id} />
        </aside>
      </main>
    </div>
  );
}
