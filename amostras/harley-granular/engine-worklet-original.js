// Granular V-twin: every grain is one full engine cycle (720°), cut between
// cycle markers found offline, and overlap-added at the current rpm's cycle
// period (PSOLA-style), so the uneven firing rhythm is never smeared.

const IDLE_RPM = 935;

class Bank {
  constructor(name, data, markersSec, sr) {
    this.name = name;
    this.data = data;
    this.m = Float64Array.from(markersSec, (t) => t * sr);
    const n = this.m.length;
    // Usable cycles c need markers c-1, c, c+1.
    this.cycles = [];
    for (let c = 1; c < n - 1; c++) {
      const period = (this.m[c + 1] - this.m[c - 1]) / 2;
      this.cycles.push({ c, rpm: (120 * sr) / period });
    }
    this.sorted = [...this.cycles].sort((a, b) => a.rpm - b.rpm);
    this.minRpm = this.sorted[0].rpm;
    this.maxRpm = this.sorted[this.sorted.length - 1].rpm;
    this.last = -1;
    this.grains = [];
    this.gain = 0;
  }

  pick(rpm, spread) {
    const s = this.sorted;
    let lo = 0, hi = s.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (s[mid].rpm < rpm) lo = mid + 1; else hi = mid;
    }
    const a = Math.max(0, lo - spread), b = Math.min(s.length - 1, lo + spread);
    let c;
    for (let tries = 0; tries < 4; tries++) {
      c = s[a + Math.floor(Math.random() * (b - a + 1))];
      if (c.c !== this.last) break;
    }
    this.last = c.c;
    return c;
  }

  sample(pos) {
    const d = this.data, i = Math.floor(pos), f = pos - i;
    const y0 = d[i - 1] || 0, y1 = d[i] || 0, y2 = d[i + 1] || 0, y3 = d[i + 2] || 0;
    const c1 = 0.5 * (y2 - y0), c2 = y0 - 2.5 * y1 + 2 * y2 - 0.5 * y3, c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
    return ((c3 * f + c2) * f + c1) * f + y1;
  }
}

class HarleyProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.banks = null;
    this.rpm = IDLE_RPM;
    this.throttle = 0;
    this.load = 0;          // smoothed throttle, decides accel vs decel material
    this.direct = false;    // direct rpm mode
    this.directRpm = IDLE_RPM;
    this.maxRpm = 6200;
    this.upRate = 1.0;      // response multiplier
    this.volume = 0.8;
    this.running = false;
    this.nextMarker = 0;    // output time (samples) of next cycle marker
    this.t = 0;
    this.period = (120 * sampleRate) / IDLE_RPM;
    this.report = 0;
    this.port.onmessage = (e) => this.onMsg(e.data);
  }

  onMsg(msg) {
    if (msg.type === 'banks') {
      this.banks = {};
      for (const b of msg.banks) this.banks[b.name] = new Bank(b.name, b.data, b.markers, sampleRate);
      this.port.postMessage({
        type: 'ready',
        ranges: Object.fromEntries(Object.values(this.banks).map((b) => [b.name, [b.minRpm, b.maxRpm, b.cycles.length]])),
      });
    } else if (msg.type === 'set') {
      if (msg.values.running && !this.running) {
        this.nextMarker = this.t;
        for (const name in this.banks || {}) this.banks[name].grains = [];
      }
      Object.assign(this, msg.values);
    }
  }

  step(dt) {
    let target;
    if (this.direct) {
      target = this.directRpm;
      const k = 1 - Math.exp(-dt * 6 * this.upRate);
      this.rpm += (target - this.rpm) * k;
    } else {
      target = IDLE_RPM + this.throttle * (this.maxRpm - IDLE_RPM);
      if (target > this.rpm) {
        // A heavy flywheel: slower at the bottom, pulls hardest in the mid range.
        const rate = (500 + 2600 * this.throttle) * this.upRate;
        this.rpm = Math.min(target, this.rpm + rate * dt);
      } else {
        const rate = (900 + 0.35 * (this.rpm - IDLE_RPM)) * this.upRate;
        this.rpm = Math.max(target, this.rpm - rate * dt);
      }
    }
    // Off-throttle material only when the engine is actually being dragged down.
    const onThrottle = this.direct ? (target >= this.rpm - 50 ? 1 : 0) : this.throttle > 0.02 ? 1 : 0;
    this.load += (onThrottle - this.load) * (1 - Math.exp(-dt * 12));
  }

  weights() {
    const { idle, accel, decel } = this.banks;
    const r = this.rpm;
    const wIdle = Math.min(1, Math.max(0, (1150 - r) / 150));
    const decelAvail = Math.min(1, Math.max(0, (r - decel.minRpm) / 350)) * Math.min(1, Math.max(0, (decel.maxRpm + 200 - r) / 200));
    const rest = 1 - wIdle;
    const wDecel = rest * (1 - this.load) * decelAvail;
    const wAccel = rest - wDecel;
    return { idle: wIdle, accel: wAccel, decel: wDecel };
  }

  spawn() {
    const P = this.period;
    const w = this.weights();
    for (const name in this.banks) {
      const bank = this.banks[name];
      // Close the grain that just finished rising: it falls over this period.
      for (const g of bank.grains) if (g.fallOut === 0) g.fallOut = P;
      bank.gain = w[name];
      if (w[name] < 0.002) continue;
      const target = Math.min(bank.maxRpm, Math.max(bank.minRpm, this.rpm));
      const cyc = bank.pick(target, name === 'idle' ? 64 : 3);
      const c = cyc.c, m = bank.m;
      bank.grains.push({
        start: this.nextMarker,
        riseOut: P, fallOut: 0,
        riseSrc0: m[c - 1], riseLen: m[c] - m[c - 1],
        fallSrc0: m[c], fallLen: m[c + 1] - m[c],
        gain: w[name],
        cycle: c,
      });
    }
  }

  process(_in, outputs) {
    const out = outputs[0];
    const L = out[0], R = out[1] || out[0];
    const n = L.length;
    if (!this.banks || !this.running) {
      L.fill(0); if (R !== L) R.fill(0);
      return true;
    }
    this.step(n / sampleRate);
    this.period = (120 * sampleRate) / this.rpm;
    const vol = this.volume;
    for (let i = 0; i < n; i++) {
      const t = this.t + i;
      while (t >= this.nextMarker) {
        this.spawn();
        this.nextMarker += this.period;
      }
      let acc = 0;
      for (const name in this.banks) {
        const bank = this.banks[name];
        for (const g of bank.grains) {
          const j = t - g.start;
          if (j < 0) continue;
          let pos, wgt;
          if (j < g.riseOut) {
            const u = j / g.riseOut;
            pos = g.riseSrc0 + u * g.riseLen;
            const s = Math.sin(u * Math.PI * 0.5); wgt = s * s;
          } else {
            if (g.fallOut === 0) continue;
            const u = (j - g.riseOut) / g.fallOut;
            if (u >= 1) { g.dead = true; continue; }
            pos = g.fallSrc0 + u * g.fallLen;
            const c = Math.cos(u * Math.PI * 0.5); wgt = c * c;
          }
          acc += bank.sample(pos) * wgt * g.gain;
        }
      }
      const y = acc * vol;
      const s = y / (1 + Math.abs(y) * 0.35); // gentle soft clip
      L[i] = s; R[i] = s;
    }
    this.t += n;
    for (const name in this.banks) {
      const b = this.banks[name];
      if (b.grains.some((g) => g.dead)) b.grains = b.grains.filter((g) => !g.dead);
    }
    if ((this.report += n) >= 1024) {
      this.report = 0;
      const w = this.weights();
      this.port.postMessage({ type: 'state', rpm: this.rpm, load: this.load, w, cycle: this.banks.accel.last });
    }
    return true;
  }
}

registerProcessor('harley-granular', HarleyProcessor);
