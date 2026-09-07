// Viewer page served at http://192.168.4.1/ .
//
// The page owns everything that does not need the camera hardware:
//   * view rotation / mirroring (CSS transform, remembered in localStorage)
//   * pulling the MJPEG stream and reconnecting it when it drops
//   * the link-quality (RSSI) readout, from /status
//   * the resolution / quality / FPS controls, which POST to /control
//
// The stream itself comes from port 81 (see STREAM_PORT in the sketch), because
// the stream handler blocks its HTTP worker for as long as a client is watching.
//
// The stream is read with fetch() + ReadableStream and painted into a <canvas>
// rather than pointed at by an <img>. An <img> showing an MJPEG stream fires
// neither `load` nor `error` when the frames simply stop arriving, so a stream
// frozen by a Wi-Fi dropout looked alive forever and never reconnected. Parsing
// the multipart body here gives an exact arrival time for every frame, and an
// AbortController that tears the dead socket down at once -- which also frees
// the matching socket on the board.

#pragma once

static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rover Camera</title>
<style>
  /* --ar is the box aspect ratio (width / height) after rotation, --img-w the
     image width that makes the rotated frame line up with the box exactly.
     JavaScript rewrites both when the view is rotated. */
  :root { --ar: 0.75; --max-w: 480px; --img-w: 133.3333%;
          --rot: 90deg; --mirror: 1; --stage-h: 60dvh; }
  body { margin: 0; min-height: 100dvh; background: #111; color: #eee;
         font-family: sans-serif; text-align: center;
         display: flex; flex-direction: column; }
  header { display: flex; flex-wrap: wrap; align-items: baseline;
           justify-content: center; gap: 0.2rem 0.7rem; padding: 0.4rem; }
  h1 { font-size: 1.1rem; margin: 0; }
  #status { font-size: 0.8rem; color: #8ab4d8; }
  /* Telemetry read-outs. tabular-nums stops the numbers jittering as they
     change, which at 10 updates a second is otherwise very distracting. */
  #rssi, #fpsActual, #age { font-size: 0.8rem; color: #888;
                            font-variant-numeric: tabular-nums; }
  #rssiChart { display: block; width: 160px; height: 32px;
               background: #1a1a1a; border: 1px solid #333; border-radius: 3px;
               align-self: center; }
  /* The stage takes whatever height is left over and centers the frame in it. */
  .stage { flex: 1; min-height: 0; display: flex;
           align-items: center; justify-content: center; }
  .viewport { position: relative; overflow: hidden; background: #000;
              aspect-ratio: var(--ar);
              width: min(100%, var(--max-w), calc(var(--stage-h) * var(--ar))); }
  .viewport canvas { position: absolute; top: 50%; left: 50%; display: block;
                     width: var(--img-w); height: auto;
                     transform: translate(-50%, -50%)
                                scaleX(var(--mirror)) rotate(var(--rot));
                     transform-origin: center center; }
  .controls { display: flex; flex-wrap: wrap; justify-content: center;
              align-items: center; gap: 0.4rem 0.9rem;
              padding: 0.5rem; font-size: 0.85rem; }
  label { display: flex; align-items: center; gap: 0.3rem; }
  button, select { font: inherit; color: #eee; background: #333;
                   border: 1px solid #555; border-radius: 4px;
                   padding: 0.25rem 0.6rem; }
  button[aria-pressed="true"] { color: #012; background: #4ab;
                                border-color: #7ce; }
  input[type="range"] { width: 7rem; }
</style>
</head>
<body>
<header id="header">
  <h1>Rover Camera</h1>
  <span id="status">接続中…</span>
  <canvas id="rssiChart"></canvas>
  <span id="rssi">電波 —</span>
  <span id="fpsActual">— fps</span>
  <span id="age">— ms</span>
</header>
<div class="stage">
  <div class="viewport">
    <canvas id="stream" width="640" height="480"></canvas>
  </div>
</div>
<div class="controls" id="controls">
  <button id="rotate" type="button">回転 90°</button>
  <button id="mirror" type="button" aria-pressed="false">左右反転</button>
  <label>解像度 <select id="framesize"></select></label>
  <label>画質 <input id="quality" type="range" min="10" max="63" step="1">
    <span id="qualityValue"></span></label>
  <label>FPS <select id="fps"></select></label>
</div>
<script>
(() => {
  'use strict';

  const VIEW_STORAGE_KEY = 'roverCamView';
  const RECONNECT_STEP_MS = 500;
  const RECONNECT_MAX_MS = 5000;
  const HEARTBEAT_MS = 3000;
  // Without this the /status fetch on a half-open socket hangs for the OS TCP
  // timeout (a minute or more on a phone) instead of failing, and the page
  // never notices that it has to reconnect.
  const STATUS_TIMEOUT_MS = 2000;
  // One timer drives the read-outs and the stall check. 100 ms is fast enough
  // for the frame-age figure to read as live while pushing the rover around.
  const TELEMETRY_MS = 100;
  const FRAME_STALL_MS = 3000;
  const FPS_WINDOW_MS = 2000;
  const AGE_WARN_MS = 500;
  const AGE_ALERT_MS = 1500;
  const MIN_STAGE_PX = 120;
  const QUARTER = { ar: 3 / 4, maxWidth: '480px', imgWidth: '133.3333%' };
  const STRAIGHT = { ar: 4 / 3, maxWidth: '640px', imgWidth: '100%' };
  const HEADER_END = [13, 10, 13, 10];  // CRLF CRLF ends a multipart part header

  // [sensor framesize value, label]. Trimmed to what the board can allocate.
  const FRAME_SIZES = [[1, '160x120'], [3, '240x176'], [5, '320x240'],
                       [6, '400x296'], [7, '480x320'], [8, '640x480'],
                       [9, '800x600'], [10, '1024x768'], [11, '1280x720'],
                       [13, '1600x1200']];
  const FPS_LIMITS = [[0, '無制限'], [30, '30'], [25, '25'], [20, '20'],
                      [15, '15'], [10, '10'], [5, '5']];
  // [minimum dBm, label, colour], strongest first. -67 dBm is the usual floor
  // for comfortable video; below -75 dBm the link is about to give up.
  const RSSI_STRONG = -55;
  const RSSI_COMFORT_FLOOR = -67;
  const RSSI_WEAK = -75;
  const RSSI_LEVELS = [[RSSI_STRONG, '強', '#6c6'],
                       [RSSI_COMFORT_FLOOR, '良', '#cc6'],
                       [RSSI_WEAK, '弱', '#e95'],
                       [-Infinity, '極弱', '#e66']];

  // One sample per heartbeat, so the chart spans HISTORY x HEARTBEAT_MS = 2 min
  // -- long enough to see where along a drive the link started to give way,
  // and coarse enough that each sample still gets a readable 4 px of width.
  const RSSI_HISTORY_MAX = 40;
  const RSSI_CHART = { w: 160, h: 32, pad: 3, min: -90, max: -30 };

  const el = (id) => document.getElementById(id);
  const canvas = el('stream');
  const context = canvas.getContext('2d');
  const chart = el('rssiChart');
  const chartContext = chart.getContext('2d');
  const textDecoder = new TextDecoder();

  let view = { rotation: 90, mirror: false };
  let streamPort = 81;
  let retries = 0;
  let reconnectTimer = null;
  let isBoardReachable = true;
  let isHeartbeatInFlight = false;
  let streamAbort = null;
  let hasFrame = false;
  // 0 = no frame has ever arrived. Only drawFrame() moves it, so the age
  // read-out keeps counting up across reconnects instead of resetting.
  let lastFrameMs = 0;
  let streamOpenedMs = 0;
  let rssiHistory = [];   // dBm per heartbeat, null while the board is silent
  let frameTimes = [];    // arrival times inside the last FPS_WINDOW_MS

  const setStatus = (text) => { el('status').textContent = text; };

  // AbortSignal.timeout() is not in every phone browser still in use.
  const timeoutSignal = (ms) => {
    const controller = new AbortController();
    setTimeout(() => controller.abort(), ms);
    return controller.signal;
  };

  // ---------- view rotation ----------

  const loadView = () => {
    try {
      const saved = JSON.parse(localStorage.getItem(VIEW_STORAGE_KEY));
      if (saved) {
        view = { rotation: saved.rotation % 360, mirror: !!saved.mirror };
      }
    } catch (err) {
      // Corrupt or unavailable storage: keep the defaults.
    }
  };

  const applyView = () => {
    const shape = view.rotation % 180 === 0 ? STRAIGHT : QUARTER;
    const root = document.documentElement.style;
    root.setProperty('--ar', shape.ar);
    root.setProperty('--max-w', shape.maxWidth);
    root.setProperty('--img-w', shape.imgWidth);
    root.setProperty('--rot', view.rotation + 'deg');
    root.setProperty('--mirror', view.mirror ? -1 : 1);
    el('rotate').textContent = '回転 ' + view.rotation + '°';
    el('mirror').setAttribute('aria-pressed', String(view.mirror));
    try {
      localStorage.setItem(VIEW_STORAGE_KEY, JSON.stringify(view));
    } catch (err) {
      // Private mode etc.: the view still works, it just is not remembered.
    }
    fitStage();
  };

  // The frame is sized off the height left between the header and the controls,
  // so it never pushes the page into scrolling.
  const fitStage = () => {
    const available = window.innerHeight - el('header').offsetHeight -
                      el('controls').offsetHeight;
    document.documentElement.style.setProperty(
        '--stage-h', Math.max(available, MIN_STAGE_PX) + 'px');
  };

  // ---------- signal strength ----------

  const gradeRssi = (rssi) => {
    const index = RSSI_LEVELS.findIndex(([floor]) => rssi >= floor);
    return { index, label: RSSI_LEVELS[index][1], color: RSSI_LEVELS[index][2] };
  };

  // The board reports what the AP measures for the connected phone; a browser
  // cannot read its own RSSI, so this is the only reading available here.
  const showSignal = (rssi) => {
    const node = el('rssi');
    if (typeof rssi !== 'number') {
      node.textContent = '電波 —';
      node.style.color = '#888';
      return;
    }
    const { index, label, color } = gradeRssi(rssi);
    const bars = RSSI_LEVELS.length - index;
    node.textContent = '▮'.repeat(bars) + '▯'.repeat(index) +
                       ' ' + rssi + ' dBm ' + label;
    node.style.color = color;
  };

  // ---------- RSSI history chart ----------

  const chartX = (index) =>
    index / (RSSI_HISTORY_MAX - 1) * (RSSI_CHART.w - 1) + 0.5;

  const chartY = (rssi) => {
    const clamped = Math.min(RSSI_CHART.max, Math.max(RSSI_CHART.min, rssi));
    const usable = RSSI_CHART.h - RSSI_CHART.pad * 2;
    return RSSI_CHART.pad +
           (RSSI_CHART.max - clamped) / (RSSI_CHART.max - RSSI_CHART.min) * usable;
  };

  const drawChart = () => {
    chartContext.clearRect(0, 0, RSSI_CHART.w, RSSI_CHART.h);

    // Guide line at the comfort floor: above it the video holds up, below it
    // the link starts costing frames. The eye only needs this one reference.
    chartContext.strokeStyle = '#444';
    chartContext.lineWidth = 1;
    chartContext.setLineDash([2, 2]);
    chartContext.beginPath();
    chartContext.moveTo(0, chartY(RSSI_COMFORT_FLOOR));
    chartContext.lineTo(RSSI_CHART.w, chartY(RSSI_COMFORT_FLOOR));
    chartContext.stroke();
    chartContext.setLineDash([]);

    // Each segment is coloured by where it ended up, so a glance shows not
    // just that the link got worse but where along the run it happened.
    // A null sample (board unreachable) leaves a deliberate gap.
    chartContext.lineWidth = 1.5;
    for (let i = 1; i < rssiHistory.length; i++) {
      const from = rssiHistory[i - 1];
      const to = rssiHistory[i];
      if (from === null || to === null) {
        continue;
      }
      chartContext.strokeStyle = gradeRssi(to).color;
      chartContext.beginPath();
      chartContext.moveTo(chartX(i - 1), chartY(from));
      chartContext.lineTo(chartX(i), chartY(to));
      chartContext.stroke();
    }

    const latest = rssiHistory[rssiHistory.length - 1];
    if (typeof latest === 'number') {
      chartContext.fillStyle = gradeRssi(latest).color;
      chartContext.beginPath();
      chartContext.arc(chartX(rssiHistory.length - 1), chartY(latest), 2,
                       0, Math.PI * 2);
      chartContext.fill();
    }
  };

  // The canvas is laid out in CSS pixels but backed at device resolution, or
  // a 1.5 px line on a phone turns into a grey smear.
  const scaleChart = () => {
    const ratio = window.devicePixelRatio || 1;
    chart.width = Math.round(RSSI_CHART.w * ratio);
    chart.height = Math.round(RSSI_CHART.h * ratio);
    chartContext.setTransform(ratio, 0, 0, ratio, 0, 0);
    drawChart();
  };

  const recordSignal = (rssi) => {
    const sample = typeof rssi === 'number' ? rssi : null;
    rssiHistory = [...rssiHistory.slice(1 - RSSI_HISTORY_MAX), sample];
    drawChart();
  };

  // ---------- frame rate and frame age ----------

  const measureFps = (now) => {
    frameTimes = frameTimes.filter((at) => now - at <= FPS_WINDOW_MS);
    if (frameTimes.length < 2) {
      return null;
    }
    const span = frameTimes[frameTimes.length - 1] - frameTimes[0];
    return span > 0 ? (frameTimes.length - 1) * 1000 / span : null;
  };

  const ageColor = (age) => {
    if (age < AGE_WARN_MS) return '#6c6';
    if (age < AGE_ALERT_MS) return '#cc6';
    return '#e66';
  };

  const showTelemetry = (now) => {
    const fps = measureFps(now);
    const node = el('fpsActual');
    node.textContent = fps === null ? '— fps' : fps.toFixed(1) + ' fps';
    node.style.color = fps === null ? '#888' : '#8ab4d8';

    const age = el('age');
    if (lastFrameMs === 0) {
      age.textContent = '— ms';
      age.style.color = '#888';
      return;
    }
    const elapsed = now - lastFrameMs;
    age.textContent = elapsed + ' ms';
    age.style.color = ageColor(elapsed);
  };

  // ---------- MJPEG stream ----------

  const concat = (head, tail) => {
    const merged = new Uint8Array(head.length + tail.length);
    merged.set(head, 0);
    merged.set(tail, head.length);
    return merged;
  };

  const indexOfSequence = (bytes, sequence) => {
    outer:
    for (let i = 0; i + sequence.length <= bytes.length; i++) {
      for (let j = 0; j < sequence.length; j++) {
        if (bytes[i + j] !== sequence[j]) {
          continue outer;
        }
      }
      return i;
    }
    return -1;
  };

  const drawFrame = async (jpeg) => {
    let bitmap;
    try {
      bitmap = await createImageBitmap(new Blob([jpeg], { type: 'image/jpeg' }));
    } catch (err) {
      return;  // a corrupt frame: skip it, the next one is usually fine
    }
    if (canvas.width !== bitmap.width || canvas.height !== bitmap.height) {
      canvas.width = bitmap.width;
      canvas.height = bitmap.height;
    }
    context.drawImage(bitmap, 0, 0);
    bitmap.close();
    lastFrameMs = Date.now();
    frameTimes = [...frameTimes, lastFrameMs];
    hasFrame = true;
    retries = 0;
  };

  // Consumes every complete part sitting in the buffer and returns the rest.
  // A part is "<boundary>Content-Type: ...\r\nContent-Length: N\r\n\r\n"
  // followed by exactly N bytes of JPEG (see STREAM_PART_HEADER in the sketch).
  const drainParts = async (buffer) => {
    while (true) {
      const headerEnd = indexOfSequence(buffer, HEADER_END);
      if (headerEnd < 0) {
        return buffer;  // part header still incomplete
      }
      const header = textDecoder.decode(buffer.subarray(0, headerEnd));
      const match = /content-length:\s*(\d+)/i.exec(header);
      const bodyStart = headerEnd + HEADER_END.length;
      if (!match) {
        buffer = buffer.slice(bodyStart);  // not a part we understand: resync
        continue;
      }
      const length = Number(match[1]);
      if (buffer.length < bodyStart + length) {
        return buffer;  // frame still incomplete
      }
      // Awaiting the decode also throttles reading, which lets TCP back-pressure
      // do the work instead of letting frames pile up in memory.
      await drawFrame(buffer.slice(bodyStart, bodyStart + length));
      buffer = buffer.slice(bodyStart + length);
    }
  };

  const readStream = async (reader, controller) => {
    let buffer = new Uint8Array(0);
    while (!controller.signal.aborted) {
      const { value, done } = await reader.read();
      if (done) {
        return;
      }
      buffer = await drainParts(concat(buffer, value));
    }
  };

  const stopStream = () => {
    if (streamAbort) {
      streamAbort.abort();
      streamAbort = null;
    }
  };

  const runStream = async (controller) => {
    // The timestamp forces a brand new connection instead of a cached one.
    const url = location.protocol + '//' + location.hostname + ':' +
                streamPort + '/stream?t=' + Date.now();
    try {
      const response = await fetch(url, {
        cache: 'no-store', signal: controller.signal,
      });
      if (!response.ok) {
        throw new Error('HTTP ' + response.status);
      }
      await readStream(response.body.getReader(), controller);
    } catch (err) {
      // Aborted by the watchdog, or superseded by a newer connect(): whoever
      // did that owns the reconnect. Every other failure falls through below.
    } finally {
      // Only the session that is still the current one may reconnect itself.
      if (streamAbort === controller) {
        streamAbort = null;
        hasFrame = false;
        scheduleReconnect();
      }
    }
  };

  const connect = () => {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
    stopStream();
    const controller = new AbortController();
    streamAbort = controller;
    streamOpenedMs = Date.now();
    runStream(controller);
  };

  // `reason` distinguishes the two ways a stream dies, which are worth telling
  // apart while flying: the socket failed, or it stayed open but went silent.
  const scheduleReconnect = (reason = '切断') => {
    if (reconnectTimer !== null) {
      return;
    }
    retries += 1;
    setStatus(reason + ' — 再接続中 (' + retries + ')');
    reconnectTimer = setTimeout(
        connect, Math.min(RECONNECT_STEP_MS * retries, RECONNECT_MAX_MS));
  };

  // A stream that stopped delivering frames but kept its socket open is the
  // case that used to be invisible: nothing errors, so only the frame clock
  // can tell. Aborting here also drops the matching dead socket on the board.
  // A freshly opened stream is measured from when it opened, since it has no
  // frame of its own yet.
  const watchdog = (now) => {
    const silentSince = Math.max(lastFrameMs, streamOpenedMs);
    if (streamAbort === null || now - silentSince <= FRAME_STALL_MS) {
      return;
    }
    stopStream();
    hasFrame = false;
    scheduleReconnect('映像停止');
  };

  const tick = () => {
    const now = Date.now();
    showTelemetry(now);
    watchdog(now);
  };

  // ---------- board status ----------

  const readStatus = async () => {
    const response = await fetch('/status', {
      cache: 'no-store', signal: timeoutSignal(STATUS_TIMEOUT_MS),
    });
    if (!response.ok) {
      throw new Error('HTTP ' + response.status);
    }
    return response.json();
  };

  const heartbeat = async () => {
    if (isHeartbeatInFlight) {
      return;  // a slow link must not let requests stack up
    }
    isHeartbeatInFlight = true;
    try {
      const status = await readStatus();
      showSignal(status.rssi);
      recordSignal(status.rssi);
      if (!isBoardReachable) {
        isBoardReachable = true;
        retries = 0;
        connect();  // the stream socket did not survive whatever happened
      }
      if (hasFrame) {
        setStatus('受信中 ' + canvas.width + 'x' + canvas.height);
      } else if (reconnectTimer === null) {
        setStatus('接続中…');
      }
    } catch (err) {
      isBoardReachable = false;
      showSignal(null);
      recordSignal(null);
      scheduleReconnect();
    } finally {
      isHeartbeatInFlight = false;
    }
  };

  // ---------- camera settings ----------

  const sendControl = async (name, value) => {
    try {
      const response = await fetch('/control?var=' + encodeURIComponent(name) +
                                   '&val=' + encodeURIComponent(value));
      if (!response.ok) {
        throw new Error(await response.text());
      }
    } catch (err) {
      setStatus('設定を適用できませんでした: ' + err.message);
    }
  };

  const fillSelect = (select, entries) => {
    select.replaceChildren(...entries.map(([value, label]) => {
      const option = document.createElement('option');
      option.value = value;
      option.textContent = label;
      return option;
    }));
  };

  const showQuality = () => {
    el('qualityValue').textContent = el('quality').value;
  };

  const loadSettings = async () => {
    try {
      const status = await readStatus();
      streamPort = status.streamPort;
      showSignal(status.rssi);
      fillSelect(el('framesize'),
                 FRAME_SIZES.filter(([value]) => value <= status.maxFramesize));
      el('framesize').value = status.framesize;
      el('quality').value = status.quality;
      el('fps').value = status.fps;
      showQuality();
    } catch (err) {
      setStatus('設定を取得できませんでした: ' + err.message);
    }
  };

  // ---------- wiring ----------

  el('rotate').addEventListener('click', () => {
    view = { ...view, rotation: (view.rotation + 90) % 360 };
    applyView();
  });
  el('mirror').addEventListener('click', () => {
    view = { ...view, mirror: !view.mirror };
    applyView();
  });
  el('framesize').addEventListener(
      'change', (event) => sendControl('framesize', event.target.value));
  el('fps').addEventListener(
      'change', (event) => sendControl('fps', event.target.value));
  el('quality').addEventListener('input', showQuality);
  el('quality').addEventListener(
      'change', (event) => sendControl('quality', event.target.value));
  window.addEventListener('resize', () => {
    fitStage();
    scaleChart();  // devicePixelRatio changes when the window moves screens
  });
  new ResizeObserver(fitStage).observe(el('controls'));

  fillSelect(el('fps'), FPS_LIMITS);
  loadView();
  applyView();
  scaleChart();
  loadSettings().then(connect);
  setInterval(heartbeat, HEARTBEAT_MS);
  setInterval(tick, TELEMETRY_MS);
})();
</script>
</body>
</html>
)rawliteral";
