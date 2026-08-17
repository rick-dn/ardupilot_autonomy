const T_CMD    = 'udl_aa_gcs/cmd';
const T_MANUAL = 'udl_aa_gcs/manual';      // node ignores this for now
const T_TELEM  = 'udl_aa_gcs/telemetry';
const T_LOG    = 'udl_aa_gcs/log';

const client = mqtt.connect('ws://localhost:9001');
client.on('error', e => console.error('MQTT error', e));
client.on('connect', () => {
  console.log('MQTT connected');
  client.subscribe([T_TELEM, T_LOG]);
});

const publish = (topic, payload) => client.publish(topic, JSON.stringify(payload));

// ── Sequences: one schema for start and abort ──────────────────────
document.querySelectorAll('.seq-btn[data-sequence]').forEach(btn => {
  btn.addEventListener('click', () =>
    publish(T_CMD, { action: 'start', name: btn.dataset.sequence }));
});

document.getElementById('abortBtn').addEventListener('click', () => {
  stopStream();
  publish(T_CMD, { action: 'abort' });
});

// ── D-pad: parked on its own topic until the node consumes it ──────
const V = 1.0, Y = 1.0, RATE_MS = 100;
const AXIS = {
  'vx+': {vx: V, vy: 0, vz: 0, yaw_rate: 0},
  'vx-': {vx:-V, vy: 0, vz: 0, yaw_rate: 0},
  'vy+': {vx: 0, vy: V, vz: 0, yaw_rate: 0},
  'vy-': {vx: 0, vy:-V, vz: 0, yaw_rate: 0},
  'vz+': {vx: 0, vy: 0, vz: V, yaw_rate: 0},
  'vz-': {vx: 0, vy: 0, vz:-V, yaw_rate: 0},
  'yaw+':{vx: 0, vy: 0, vz: 0, yaw_rate: Y},
  'yaw-':{vx: 0, vy: 0, vz: 0, yaw_rate:-Y},
};
let streamTimer = null, activeBtn = null;

function startStream(cmd, btn){
  stopStream();
  activeBtn = btn; btn.classList.add('held');
  const send = () => publish(T_MANUAL, { type:'set_velocity', ...cmd });
  send();
  streamTimer = setInterval(send, RATE_MS);
}
function stopStream(){
  if (streamTimer){ clearInterval(streamTimer); streamTimer = null; }
  if (activeBtn){ activeBtn.classList.remove('held'); activeBtn = null; }
  publish(T_MANUAL, { type:'stop_velocity' });
}

document.querySelectorAll('.dbtn[data-vel]').forEach(btn => {
  const cmd = AXIS[btn.dataset.vel];
  btn.addEventListener('pointerdown',  e => { e.preventDefault(); startStream(cmd, btn); });
  btn.addEventListener('pointerup',    e => { e.preventDefault(); stopStream(); });
  btn.addEventListener('pointerleave', () => { if (activeBtn === btn) stopStream(); });
  btn.addEventListener('pointercancel',() => { if (activeBtn === btn) stopStream(); });
});

// ── Inbound ────────────────────────────────────────────────────────
const n = (v, d=1) => (typeof v === 'number' ? v.toFixed(d) : '—');

// The log is the record of the run, so it accumulates rather than replacing
// itself. Capped so a long flight can't grow the DOM without bound.
const MAX_LOG_LINES = 300;
const logEl = document.getElementById('log');

function appendLog(text){
  const stamp = new Date().toTimeString().slice(0, 8);
  const line = document.createElement('div');
  line.className = 'log-line';
  line.textContent = `${stamp}  ${text}`;
  logEl.appendChild(line);
  while (logEl.childElementCount > MAX_LOG_LINES) logEl.removeChild(logEl.firstChild);
  logEl.scrollTop = logEl.scrollHeight;
}

client.on('message', (topic, raw) => {
  let m; try { m = JSON.parse(raw.toString()); } catch(e){ return; }

  if (topic === T_TELEM){
    document.getElementById('lineState').textContent =
      `mode ${m.mode || '—'} · mission ${m.mission ?? 'idle'}`;
    document.getElementById('linePose').textContent =
      `E ${n(m.east, 2)}  N ${n(m.north, 2)}  U ${n(m.up, 2)}  yaw ${n(m.yaw_deg, 0)}°`;
    document.getElementById('lineVel').textContent =
      `vE ${n(m.vel_east, 2)}  vN ${n(m.vel_north, 2)}  vU ${n(m.vel_up, 2)}`;
    document.getElementById('lineHealth').textContent =
      `batt ${n(m.batt, 0)}% · ${m.armed ? 'ARMED' : 'disarmed'}` +
      ` · ${m.in_air ? 'IN AIR' : 'ground'}`;
  }

  if (topic === T_LOG){
    appendLog(m.text ?? '');
  }
});
