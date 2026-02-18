// ===== WebSocket =====
let ws = null;
let reconnectTimer = null;

function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws/web`);

  ws.onopen = () => {
    setStatus('wsStatus', 'Server: Connected', true);
    addLog('Connected to server', 'log-event');
  };

  ws.onclose = () => {
    setStatus('wsStatus', 'Server: Disconnected', false);
    setStatus('deviceStatus', 'Device: Disconnected', false);
    addLog('Disconnected from server', 'log-error');
    reconnectTimer = setTimeout(connectWS, 3000);
  };

  ws.onerror = () => {};

  ws.onmessage = (e) => {
    let msg;
    try { msg = JSON.parse(e.data); } catch { return; }
    handleMessage(msg);
  };
}

function sendWS(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

// ===== Message Handler =====
function handleMessage(msg) {
  switch (msg.type) {
    case 'status':
      setStatus('deviceStatus',
        msg.esp32Connected ? 'Device: Connected' : 'Device: Disconnected',
        msg.esp32Connected);
      break;

    case 'event':
      handleEvent(msg);
      break;

    case 'chat':
      addChatMessage(msg.role, msg.text);
      break;

    case 'actions':
      if (msg.actions) {
        msg.actions.forEach(a => {
          addLog(`AI called: ${a.tool}(${JSON.stringify(a.args)})`, 'log-cmd');
        });
      }
      break;

    case 'command_result':
      addLog(`Command result: ${msg.status}`, msg.status === 'ok' ? 'log-event' : 'log-error');
      break;
  }
}

function handleEvent(msg) {
  const evt = msg.event;
  const data = msg.data || {};

  switch (evt) {
    case 'distance':
      updateDistance(data.distance_mm);
      break;
    case 'object_detected':
      addLog(`Object detected at ${data.distance_mm}mm`, 'log-event');
      break;
    case 'recognition_start':
      addLog('Recognition started...', 'log-event');
      break;
    case 'recognition_result':
      addLog(`Recognized: ${data.result}`, 'log-event');
      addChatMessage('system', `Object recognized: ${data.result}`);
      break;
    case 'device_info':
      if (data.ip) {
        addLog(`Device IP: ${data.ip}`, 'log-event');
        startCameraFeed(data.ip);
      }
      break;
    default:
      addLog(`Event: ${evt} ${JSON.stringify(data)}`, 'log-event');
  }
}

// ===== Camera Feed =====
let camDeviceIp = null;
let camBusy = false;
let camCapturing = false;
let camFc = 0;
let camFt = Date.now();
let camPollTimer = null;
let camStatusTimer = null;

function startCameraFeed(ip) {
  camDeviceIp = ip;
  document.getElementById('camStatus').textContent = `http://${ip}`;
  addLog(`Camera feed started: http://${ip}/capture`, 'log-event');
  nextCamFrame();
  camStatusTimer = setInterval(pollCamStatus, 500);
}

function nextCamFrame() {
  if (!camDeviceIp) return;
  if (camBusy || camCapturing) {
    camPollTimer = setTimeout(nextCamFrame, camCapturing ? 300 : 30);
    return;
  }
  camBusy = true;
  const img = new Image();
  img.onload = () => {
    document.getElementById('camFeed').src = img.src;
    camBusy = false;
    camFc++;
    const now = Date.now();
    if (now - camFt >= 1000) {
      document.getElementById('camFps').textContent = `${camFc} fps`;
      camFc = 0; camFt = now;
    }
    camPollTimer = setTimeout(nextCamFrame, 80); // ~12fps
  };
  img.onerror = () => {
    camBusy = false;
    camPollTimer = setTimeout(nextCamFrame, 500);
  };
  img.src = `http://${camDeviceIp}/capture?t=${Date.now()}`;
}

async function pollCamStatus() {
  if (!camDeviceIp) return;
  try {
    const res = await fetch(`http://${camDeviceIp}/status`, { cache: 'no-store' });
    const data = await res.json();
    camCapturing = data.capturing;
    const overlay = document.getElementById('camOverlay');
    const statusEl = document.getElementById('camStatus');
    if (camCapturing) {
      overlay.classList.add('on');
      statusEl.textContent = 'AI Recognizing...';
    } else {
      overlay.classList.remove('on');
      statusEl.textContent = `http://${camDeviceIp}`;
    }
  } catch (e) {
    // device unreachable, keep trying
  }
}

// ===== Distance =====
const distHistory = [];
const MAX_DIST_POINTS = 60;
const MAX_DIST_DISPLAY = 2000;

function updateDistance(mm) {
  document.getElementById('distValue').textContent = mm;

  const pct = Math.min(mm / MAX_DIST_DISPLAY * 100, 100);
  document.getElementById('distBar').style.width = pct + '%';

  distHistory.push(mm);
  if (distHistory.length > MAX_DIST_POINTS) distHistory.shift();
  drawDistGraph();
}

function drawDistGraph() {
  const canvas = document.getElementById('distGraph');
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;

  ctx.clearRect(0, 0, w, h);

  if (distHistory.length < 2) return;

  // Grid
  ctx.strokeStyle = '#333';
  ctx.lineWidth = 0.5;
  for (let y = 0; y < h; y += h / 4) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  // Line
  ctx.strokeStyle = '#4caf50';
  ctx.lineWidth = 2;
  ctx.beginPath();

  const step = w / (MAX_DIST_POINTS - 1);
  const startX = w - (distHistory.length - 1) * step;

  for (let i = 0; i < distHistory.length; i++) {
    const x = startX + i * step;
    const val = Math.min(distHistory[i], MAX_DIST_DISPLAY);
    const y = h - (val / MAX_DIST_DISPLAY) * h;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

// ===== UI Helpers =====
function setStatus(id, text, connected) {
  const el = document.getElementById(id);
  el.textContent = text;
  el.className = 'status-badge ' + (connected ? 'connected' : 'disconnected');
}

function addChatMessage(role, text) {
  const log = document.getElementById('chatLog');
  const div = document.createElement('div');
  div.className = 'chat-msg ' + role;

  if (role !== 'system') {
    const roleEl = document.createElement('div');
    roleEl.className = 'role';
    roleEl.textContent = role === 'user' ? 'You' : 'AI';
    div.appendChild(roleEl);
  }

  const textEl = document.createElement('div');
  textEl.textContent = text;
  div.appendChild(textEl);

  log.appendChild(div);
  log.scrollTop = log.scrollHeight;
}

function addLog(text, cls) {
  const log = document.getElementById('eventLog');
  const div = document.createElement('div');
  div.className = 'log-entry';

  const time = new Date().toLocaleTimeString('ko-KR', { hour12: false });
  div.innerHTML = `<span class="log-time">${time}</span><span class="${cls || ''}">${text}</span>`;

  log.appendChild(div);
  log.scrollTop = log.scrollHeight;

  // Keep max 100 entries
  while (log.children.length > 100) {
    log.removeChild(log.firstChild);
  }
}

// ===== Chat Input =====
const chatInput = document.getElementById('chatInput');
const sendBtn = document.getElementById('sendBtn');

function sendChat() {
  const text = chatInput.value.trim();
  if (!text) return;
  chatInput.value = '';
  sendWS({ type: 'chat', text });
}

sendBtn.addEventListener('click', sendChat);
chatInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') sendChat();
});

// ===== Voice Input =====
const voiceBtn = document.getElementById('voiceBtn');
let recognition = null;

if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {
  const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  recognition = new SpeechRecognition();
  recognition.lang = 'ko-KR';
  recognition.continuous = false;
  recognition.interimResults = false;

  recognition.onresult = (e) => {
    const text = e.results[0][0].transcript;
    chatInput.value = text;
    sendChat();
  };

  recognition.onend = () => {
    voiceBtn.classList.remove('recording');
    voiceBtn.textContent = 'MIC';
  };

  recognition.onerror = () => {
    voiceBtn.classList.remove('recording');
    voiceBtn.textContent = 'MIC';
  };

  voiceBtn.addEventListener('click', () => {
    if (voiceBtn.classList.contains('recording')) {
      recognition.stop();
    } else {
      recognition.start();
      voiceBtn.classList.add('recording');
      voiceBtn.textContent = 'REC';
    }
  });
} else {
  voiceBtn.disabled = true;
  voiceBtn.title = 'Speech recognition not supported';
}

// ===== Manual Control (press & hold) =====
const speed = () => parseInt(document.getElementById('speedSlider').value);

document.getElementById('speedSlider').addEventListener('input', (e) => {
  document.getElementById('speedValue').textContent = e.target.value;
});

document.querySelectorAll('.pad-btn').forEach(btn => {
  const cmd = btn.dataset.cmd;
  if (!cmd) return;

  const startMove = (e) => {
    e.preventDefault();
    btn.classList.add('active');
    if (cmd === 'stop_motors') {
      sendWS({ type: 'command', command: cmd, params: {} });
    } else {
      sendWS({ type: 'command', command: cmd, params: { speed: speed() } });
    }
  };

  const stopMove = (e) => {
    e.preventDefault();
    btn.classList.remove('active');
    if (cmd !== 'stop_motors') {
      sendWS({ type: 'command', command: 'stop_motors', params: {} });
    }
  };

  // Mouse
  btn.addEventListener('mousedown', startMove);
  btn.addEventListener('mouseup', stopMove);
  btn.addEventListener('mouseleave', stopMove);
  // Touch
  btn.addEventListener('touchstart', startMove);
  btn.addEventListener('touchend', stopMove);
  btn.addEventListener('touchcancel', stopMove);
});

// ===== Autonomous Mode Toggle =====
document.getElementById('autoModeToggle').addEventListener('change', (e) => {
  sendWS({ type: 'command', command: 'set_autonomous_mode', params: { enabled: e.target.checked } });
  addLog(`Autonomous mode: ${e.target.checked ? 'ON' : 'OFF'}`, 'log-cmd');
});

// ===== Photo Button =====
document.getElementById('photoBtn').addEventListener('click', () => {
  sendWS({ type: 'command', command: 'take_photo_and_recognize', params: {} });
  addLog('Photo & recognize requested', 'log-cmd');
});

// ===== Clear Log =====
document.getElementById('clearLogBtn').addEventListener('click', () => {
  document.getElementById('eventLog').innerHTML = '';
});

// ===== Init =====
connectWS();
