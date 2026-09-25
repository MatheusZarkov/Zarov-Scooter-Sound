// Motor da Hayabusa com o virabrequim imposto: sem resolvedor de corpo rigido.
// Gas, combustao, valvulas e sintetizador sao o codigo original do engine-sim (MIT).
import { buildEngine } from './engine-sim/src/builder/buildEngine';
import { PistonEngineSimulator } from './engine-sim/src/sim/pistonEngineSimulator';
import { Vehicle } from './engine-sim/src/engine/vehicle';
import { Transmission } from './engine-sim/src/engine/transmission';
import { setGasKernels } from './engine-sim/src/engine/gasSystem';
import { hayabusa } from './engine-sim/src/engines/hayabusa';

const RPM_TO_RADS = (2 * Math.PI) / 60;

export interface Opcoes {
  simFreq?: number;
  fluidSteps?: number;
  audioRate?: number;
  ir?: Int16Array | null;
}

export class SomCinematico {
  readonly sim = new PistonEngineSimulator();
  readonly engine = buildEngine(hayabusa.engine());
  readonly audioRate: number;
  private readonly s: any;
  private px: Float64Array;
  private py: Float64Array;

  constructor(o: Opcoes = {}) {
    this.audioRate = o.audioRate ?? 44100;
    setGasKernels(null);
    const vehicle = new Vehicle();
    vehicle.initialize(hayabusa.vehicle());
    const transmission = new Transmission();
    transmission.initialize(hayabusa.transmission());

    this.sim.initialize();
    this.sim.setSimulationFrequency(o.simFreq ?? this.engine.getSimulationFrequency());
    this.sim.setAudioSampleRate(this.audioRate);
    this.sim.setFluidSimulationSteps(o.fluidSteps ?? 8);
    this.sim.loadSimulation(this.engine, vehicle, transmission);
    this.s = this.sim as any;

    // mesmos parametros de audio que o site aplica para a Hayabusa
    const p = this.sim.getSynthesizer().audioParameters;
    p.dF_F_mix = 0.00407;
    p.airNoise = 0.292;
    p.inputSampleNoise = 0.062;
    if (o.ir) {
      for (let c = 0; c < this.engine.getExhaustSystemCount(); ++c) {
        const ex = this.engine.getExhaustSystem(c);
        this.sim.getSynthesizer().initializeImpulseResponse(o.ir, ex.getImpulseResponseVolume() * 0.71, c);
      }
    }

    // No port, a posicao "global" do moente nao gira com o virabrequim (so e usada
    // na montagem, com angulo zero). Aqui o virabrequim gira, entao aplica a rotacao.
    for (let i = 0; i < this.engine.getCrankshaftCount(); ++i) {
      const c: any = this.engine.getCrankshaft(i);
      const tmp = { x: 0, y: 0 };
      c.getRodJournalPositionGlobal = (j: number, out: { x: number; y: number }) => {
        c.getRodJournalPositionLocal(j, tmp);
        c.body.localToWorld(tmp.x, tmp.y, out);
      };
    }

    this.engine.getIgnitionModule().enabled = true;
    const n = this.engine.getCylinderCount();
    this.px = new Float64Array(n);
    this.py = new Float64Array(n);
    this.posicionar(0);
    for (let i = 0; i < n; ++i) {
      this.px[i] = this.engine.getPiston(i).body.p_x;
      this.py[i] = this.engine.getPiston(i).body.p_y;
    }
  }

  private posicionar(dt: number): void {
    const n = this.engine.getCylinderCount();
    for (let i = 0; i < n; ++i) this.s.placeCylinder(i);
    if (dt <= 0) return;
    for (let i = 0; i < n; ++i) {
      const b = this.engine.getPiston(i).body;
      b.v_x = (b.p_x - this.px[i]) / dt;
      b.v_y = (b.p_y - this.py[i]) / dt;
      this.px[i] = b.p_x;
      this.py[i] = b.p_y;
    }
  }

  /** Avanca um quadro de `dt` segundos com rpm e acelerador constantes; devolve as amostras de audio. */
  quadro(dt: number, rpm: number, acelerador: number): Int16Array {
    const sim = this.sim;
    const engine = this.engine;
    sim.externalAudioLatency = sim.getTargetSynthesizerLatency();
    sim.startFrame(dt);
    const passos = Math.round(dt * sim.getSimulationFrequency());
    const h = sim.getTimestep();
    const w = -rpm * RPM_TO_RADS; // gira no sentido horario, como o motor de partida
    engine.setSpeedControl(acelerador);
    const crank = engine.getOutputCrankshaft();

    for (let k = 0; k < passos; ++k) {
      crank.body.v_theta = w;
      crank.body.theta += w * h;
      for (let i = 0; i < engine.getCrankshaftCount(); ++i) {
        engine.getCrankshaft(i).body.theta = crank.body.theta;
        engine.getCrankshaft(i).body.v_theta = w;
      }
      this.posicionar(h);
      engine.update(h);
      this.s.updateFilteredEngineSpeed(h);
      crank.resetAngle();
      this.s.simulateStepInternal();
      this.s.writeToSynthesizer();
    }
    sim.endFrame();
    const n = sim.getSynthesizer().queuedOutputSamples();
    const out = new Int16Array(n);
    sim.readAudioOutput(n, out);
    return out;
  }
}
