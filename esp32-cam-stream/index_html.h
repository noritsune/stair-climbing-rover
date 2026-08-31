// Viewer page served at http://192.168.4.1/ .
//
// The page owns everything that does not need the camera hardware:
//   * view rotation / mirroring (CSS transform, remembered in localStorage)
//   * reconnecting the MJPEG stream when it drops
//   * the resolution / quality / FPS controls, which POST to /control
//
// The stream itself comes from port 81 (see STREAM_PORT in the sketch), because
// the stream handler blocks its HTTP worker for as long as a client is watching.

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
  header { display: flex; align-items: baseline; justify-content: center;
           gap: 0.6rem; padding: 0.4rem; }
  h1 { font-size: 1.1rem; margin: 0; }
  #status { font-size: 0.8rem; color: #8ab4d8; }
  /* The stage takes whatever height is left over and centers the frame in it. */
  .stage { flex: 1; min-height: 0; display: flex;
           align-items: center; justify-content: center; }
  .viewport { position: relative; overflow: hidden; background: #000;
              aspect-ratio: var(--ar);
              width: min(100%, var(--max-w), calc(var(--stage-h) * var(--ar))); }
  .viewport img { position: absolute; top: 50%; left: 50%;
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
</header>
<div class="stage">
  <div class="viewport">
    <img id="stream" alt="camera stream">
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
  const MIN_STAGE_PX = 120;
  const QUARTER = { ar: 3 / 4, maxWidth: '480px', imgWidth: '133.3333%' };
  const STRAIGHT = { ar: 4 / 3, maxWidth: '640px', imgWidth: '100%' };

  // [sensor framesize value, label]. Trimmed to what the board can allocate.
  const FRAME_SIZES = [[1, '160x120'], [3, '240x176'], [5, '320x240'],
                       [6, '400x296'], [7, '480x320'], [8, '640x480'],
                       [9, '800x600'], [10, '1024x768'], [11, '1280x720'],
                       [13, '1600x1200']];
  const FPS_LIMITS = [[0, '無制限'], [30, '30'], [25, '25'], [20, '20'],
                      [15, '15'], [10, '10'], [5, '5']];

  const el = (id) => document.getElementById(id);
  const stream = el('stream');

  let view = { rotation: 90, mirror: false };
  let streamPort = 81;
  let retries = 0;
  let reconnectTimer = null;
  let isBoardReachable = true;

  const setStatus = (text) => { el('status').textContent = text; };

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

  // ---------- stream connection ----------

  const connect = () => {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
    // The timestamp forces a brand new connection instead of a cached one.
    stream.src = location.protocol + '//' + location.hostname + ':' +
                 streamPort + '/stream?t=' + Date.now();
  };

  const scheduleReconnect = () => {
    if (reconnectTimer !== null) {
      return;
    }
    retries += 1;
    setStatus('切断 — 再接続中 (' + retries + ')');
    reconnectTimer = setTimeout(
        connect, Math.min(RECONNECT_STEP_MS * retries, RECONNECT_MAX_MS));
  };

  // An MJPEG <img> reports nothing per frame, so /status doubles as the
  // heartbeat: it catches a reboot or a Wi-Fi drop that leaves the stream
  // socket silently frozen.
  const heartbeat = async () => {
    try {
      const response = await fetch('/status', { cache: 'no-store' });
      if (!response.ok) {
        throw new Error('HTTP ' + response.status);
      }
      if (!isBoardReachable) {
        isBoardReachable = true;
        retries = 0;
        connect();  // the stream socket did not survive whatever happened
      }
      if (stream.naturalWidth > 0) {
        retries = 0;
        setStatus('受信中 ' + stream.naturalWidth + 'x' + stream.naturalHeight);
      } else {
        setStatus('接続中…');
      }
    } catch (err) {
      isBoardReachable = false;
      scheduleReconnect();
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
      const response = await fetch('/status', { cache: 'no-store' });
      if (!response.ok) {
        throw new Error('HTTP ' + response.status);
      }
      const status = await response.json();
      streamPort = status.streamPort;
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
  stream.addEventListener('error', scheduleReconnect);
  window.addEventListener('resize', fitStage);
  new ResizeObserver(fitStage).observe(el('controls'));

  fillSelect(el('fps'), FPS_LIMITS);
  loadView();
  applyView();
  loadSettings().then(connect);
  setInterval(heartbeat, HEARTBEAT_MS);
})();
</script>
</body>
</html>
)rawliteral";
