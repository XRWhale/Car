require('dotenv').config();
const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const path = require('path');
const { AIController } = require('./ai-controller');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const PORT = process.env.PORT || 3000;

// Static files
app.use(express.static(path.join(__dirname, 'public')));

// State
let esp32Client = null;
const webClients = new Set();
const pendingCommands = new Map();
let commandCounter = 0;

// Send command to ESP32 and wait for response
function sendCommandToEsp32(command, params = {}) {
  return new Promise((resolve, reject) => {
    if (!esp32Client || esp32Client.readyState !== WebSocket.OPEN) {
      reject(new Error('ESP32 not connected'));
      return;
    }

    const id = `cmd_${++commandCounter}`;
    const timeoutMs = command === 'take_photo_and_recognize' ? 60000 : 10000;

    const timer = setTimeout(() => {
      pendingCommands.delete(id);
      reject(new Error(`Command timeout: ${command}`));
    }, timeoutMs);

    pendingCommands.set(id, { resolve, reject, timer });
    esp32Client.send(JSON.stringify({ id, command, params }));
    console.log(`[ESP32 >>] ${command}`, params);
  });
}

// Broadcast to all web clients
function broadcastToWeb(msg) {
  const data = typeof msg === 'string' ? msg : JSON.stringify(msg);
  for (const ws of webClients) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(data);
    }
  }
}

// AI Controller
const aiController = new AIController(sendCommandToEsp32);

// WebSocket connections
wss.on('connection', (ws, req) => {
  const urlPath = req.url;

  if (urlPath === '/ws/esp32') {
    // --- ESP32 connection ---
    console.log('[ESP32] Connected');
    esp32Client = ws;
    broadcastToWeb({ type: 'status', esp32Connected: true });

    ws.on('message', (raw) => {
      let msg;
      try {
        msg = JSON.parse(raw.toString());
      } catch (e) {
        console.error('[ESP32] Invalid JSON:', raw.toString());
        return;
      }

      if (msg.type === 'response') {
        // Command response
        const pending = pendingCommands.get(msg.id);
        if (pending) {
          clearTimeout(pending.timer);
          pendingCommands.delete(msg.id);
          pending.resolve(msg);
        }
      } else if (msg.type === 'event') {
        // Forward event to web clients
        console.log(`[ESP32 event] ${msg.event}`, msg.data || '');
        broadcastToWeb(msg);
      }
    });

    ws.on('close', () => {
      console.log('[ESP32] Disconnected');
      if (esp32Client === ws) esp32Client = null;
      broadcastToWeb({ type: 'status', esp32Connected: false });
    });

    ws.on('error', (err) => {
      console.error('[ESP32] Error:', err.message);
    });

  } else {
    // --- Web client connection ---
    console.log('[Web] Client connected');
    webClients.add(ws);

    // Send current status
    ws.send(JSON.stringify({
      type: 'status',
      esp32Connected: esp32Client !== null && esp32Client.readyState === WebSocket.OPEN
    }));

    ws.on('message', async (raw) => {
      let msg;
      try {
        msg = JSON.parse(raw.toString());
      } catch (e) {
        console.error('[Web] Invalid JSON:', raw.toString());
        return;
      }

      if (msg.type === 'chat') {
        // AI chat message
        console.log(`[Web chat] ${msg.text}`);
        broadcastToWeb({ type: 'chat', role: 'user', text: msg.text });

        try {
          const result = await aiController.processMessage(msg.text);
          broadcastToWeb({ type: 'chat', role: 'assistant', text: result.reply });
          if (result.actions && result.actions.length > 0) {
            broadcastToWeb({ type: 'actions', actions: result.actions });
          }
        } catch (err) {
          console.error('[AI] Error:', err.message);
          broadcastToWeb({
            type: 'chat',
            role: 'assistant',
            text: `Error: ${err.message}`
          });
        }

      } else if (msg.type === 'command') {
        // Direct command (manual control) — fire-and-forget, no await
        console.log(`[Web cmd] ${msg.command}`, msg.params || {});
        if (esp32Client && esp32Client.readyState === WebSocket.OPEN) {
          const id = `cmd_${++commandCounter}`;
          esp32Client.send(JSON.stringify({
            id,
            command: msg.command,
            params: msg.params || {}
          }));
        } else {
          ws.send(JSON.stringify({
            type: 'command_result',
            status: 'error',
            data: { message: 'ESP32 not connected' }
          }));
        }
      }
    });

    ws.on('close', () => {
      console.log('[Web] Client disconnected');
      webClients.delete(ws);
    });

    ws.on('error', (err) => {
      console.error('[Web] Error:', err.message);
    });
  }
});

server.listen(PORT, () => {
  console.log(`\n========================================`);
  console.log(`  Robot Control Server`);
  console.log(`  http://localhost:${PORT}`);
  console.log(`  WS ESP32: ws://localhost:${PORT}/ws/esp32`);
  console.log(`  WS Web:   ws://localhost:${PORT}/ws`);
  console.log(`========================================\n`);
});
