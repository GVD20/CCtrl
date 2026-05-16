import { existsSync, readFileSync, statSync } from 'node:fs';
import { createServer as createHttpServer } from 'node:http';
import { createServer as createHttpsServer } from 'node:https';
import { createConnection } from 'node:net';
import { networkInterfaces } from 'node:os';
import { extname, join, normalize } from 'node:path';
import process from 'node:process';

import { WebSocketServer } from 'ws';

const DEFAULT_PORT = Number(process.env.PORT ?? 8787);
const HOST = process.env.HOST ?? '0.0.0.0';
const SERVE_DIST = process.argv.includes('--serve-dist');
const DIST_DIR = join(process.cwd(), 'dist');
const RUNTIME_CONFIG_PATH = join(process.cwd(), 'server', 'runtime-config.json');

const XR_PACKET_MAGIC = 0x31525843;
const XR_PACKET_SIZE = 96;
const BRIDGE_RECONNECT_MS = 800;

const tlsKeyPath = process.env.TLS_KEY_PATH ?? join(process.cwd(), 'certs', 'localhost-key.pem');
const tlsCertPath = process.env.TLS_CERT_PATH ?? join(process.cwd(), 'certs', 'localhost.pem');
const tlsCaPath = process.env.TLS_CA_PATH ?? join(process.cwd(), 'certs', 'quest-dev-root-ca.pem');
const useTls = existsSync(tlsKeyPath) && existsSync(tlsCertPath);

const clients = new Set();
const sessionCache = new Map();

const latestFrameSummary = {
  sessionId: '',
  frameSeq: 0,
  questSentAtMs: 0,
  receivedAtMs: 0,
  relayedAtMs: 0,
  keyFlags: 0,
  rightConnected: false,
  joyX: 50,
  joyY: 50,
  relPositionMm: [0, 0, 0],
  relQuaternionWxyz: [1, 0, 0, 0]
};

const relayStats = {
  framesReceived: 0,
  framesRelayed: 0,
  framesDroppedSeq: 0,
  bridgeWriteErrors: 0,
  lastBridgeError: '',
  lastPacketBytes: 0
};

const relayTiming = {
  questRxTimesMs: [],
  bridgeTxTimesMs: []
};

const bridgeState = {
  socket: null,
  connecting: false,
  reconnectTimer: null,
  host: '',
  port: 0,
  connectedAtMs: 0,
  lastFrameSeq: 0,
  lastWriteAtMs: 0,
  lastDisconnectAtMs: 0,
  lastError: ''
};

const httpHandler = (req, res) => {
  const url = new URL(req.url ?? '/', `${useTls ? 'https' : 'http'}://${req.headers.host ?? 'localhost'}`);

  if (url.pathname === '/health' || url.pathname === '/status') {
    const config = loadRuntimeConfig();
    sendJson(res, 200, {
      ok: true,
      transport: useTls ? 'https+wss' : 'http+ws',
      serveDist: SERVE_DIST,
      clients: clients.size,
      activeSessions: sessionCache.size,
      sessions: url.pathname === '/status' ? [...sessionCache.values()] : undefined,
      relay: buildRelayStatus(config),
      accessUrls: listAccessUrls(DEFAULT_PORT, useTls)
    });
    return;
  }

  if (url.pathname === '/runtime-config.json') {
    sendJson(res, 200, loadRuntimeConfig());
    return;
  }

  if (!SERVE_DIST) {
    sendJson(res, 200, {
      ok: true,
      message: 'WebSocket receiver is running. Start Vite separately for local frontend dev.',
      websocketPath: '/ws'
    });
    return;
  }

  serveStaticAsset(url.pathname, res);
};

const server = useTls
  ? createHttpsServer(
    {
      key: readFileSync(tlsKeyPath),
      cert: readFileSync(tlsCertPath),
      ca: existsSync(tlsCaPath) ? readFileSync(tlsCaPath) : undefined
    },
    httpHandler
  )
  : createHttpServer(httpHandler);

const wss = new WebSocketServer({ noServer: true });

server.on('upgrade', (request, socket, head) => {
  const url = new URL(request.url ?? '/', `${useTls ? 'https' : 'http'}://${request.headers.host ?? 'localhost'}`);
  if (url.pathname !== '/ws') {
    socket.destroy();
    return;
  }

  wss.handleUpgrade(request, socket, head, (ws) => {
    wss.emit('connection', ws, request);
  });
});

wss.on('connection', (ws, request) => {
  clients.add(ws);
  const origin = request.headers.origin ?? 'unknown-origin';
  console.log(`[ws] connected (${clients.size} clients) from ${origin}`);

  ws.on('message', (buffer) => {
    try {
      const payload = JSON.parse(buffer.toString());
      handleMessage(payload);
    } catch (error) {
      console.error('[ws] invalid message', error);
    }
  });

  ws.on('close', () => {
    clients.delete(ws);
    console.log(`[ws] disconnected (${clients.size} clients remain)`);
  });
});

server.listen(DEFAULT_PORT, HOST, () => {
  const scheme = useTls ? 'https' : 'http';
  const wsScheme = useTls ? 'wss' : 'ws';
  console.log(`[server] listening on ${scheme}://${HOST}:${DEFAULT_PORT}`);
  console.log(`[server] websocket endpoint ${wsScheme}://${HOST}:${DEFAULT_PORT}/ws`);
  if (SERVE_DIST) {
    console.log('[server] serving ./dist for Quest-ready same-origin hosting');
  } else {
    console.log('[server] static hosting disabled; run `npm run dev` separately for frontend work');
  }

  if (SERVE_DIST && !useTls) {
    console.warn('[server] HTTPS is not active. Quest Browser will fail on https://... with ERR_SSL_PROTOCOL_ERROR.');
    console.warn('[server] Generate certs with `npm run certs:generate`, then restart `npm run serve`.');
  }

  ensureBridgeConnection();
});

function handleMessage(payload) {
  switch (payload?.type) {
    case 'hello':
      console.log(`[hello] protocol=${payload.protocolVersion} xrSupported=${payload.xrSupported}`);
      break;

    case 'session_start':
      sessionCache.set(payload.sessionId, {
        sessionId: payload.sessionId,
        startedAtMs: payload.startedAtMs,
        referenceSpace: payload.referenceSpace,
        sessionMode: payload.sessionMode,
        lastFrameSeq: 0,
        lastSeenAtMs: Date.now(),
        rightConnected: false,
        relayOk: false
      });
      console.log(`[session_start] ${payload.sessionId} (${payload.referenceSpace})`);
      break;

    case 'pose_frame':
      handlePoseFrame(payload);
      break;

    case 'session_end':
      console.log(`[session_end] ${payload.sessionId} reason=${payload.reason}`);
      sessionCache.delete(payload.sessionId);
      break;

    default:
      console.log(`[ws] unhandled type=${payload?.type ?? 'unknown'}`);
      break;
  }
}

function handlePoseFrame(payload) {
  relayStats.framesReceived += 1;
  appendTiming(relayTiming.questRxTimesMs, Date.now(), 240);

  const sessionInfo = sessionCache.get(payload.sessionId) ?? {
    sessionId: payload.sessionId,
    startedAtMs: null,
    referenceSpace: payload.referenceSpace,
    sessionMode: payload.sessionMode,
    lastFrameSeq: 0,
    lastSeenAtMs: Date.now(),
    rightConnected: false,
    relayOk: false
  };

  const nextSeq = Number(payload.frameSeq ?? 0) >>> 0;
  if (sessionInfo.lastFrameSeq > 0 && nextSeq > sessionInfo.lastFrameSeq + 1) {
    relayStats.framesDroppedSeq += nextSeq - sessionInfo.lastFrameSeq - 1;
  }

  sessionInfo.lastFrameSeq = nextSeq;
  sessionInfo.lastSeenAtMs = Date.now();
  sessionInfo.rightConnected = Boolean(payload.rightController?.connected);

  const relayResult = relayPoseFrame(payload);
  sessionInfo.relayOk = relayResult.ok;
  sessionInfo.lastRelayError = relayResult.error;
  sessionInfo.lastKeyFlags = Number(payload.rightController?.keyFlags ?? 0);

  latestFrameSummary.sessionId = String(payload.sessionId ?? '');
  latestFrameSummary.frameSeq = sessionInfo.lastFrameSeq;
  latestFrameSummary.questSentAtMs = Number(payload.sentAtMs ?? 0) || 0;
  latestFrameSummary.receivedAtMs = sessionInfo.lastSeenAtMs;
  latestFrameSummary.relayedAtMs = relayResult.ok ? Date.now() : latestFrameSummary.relayedAtMs;
  latestFrameSummary.keyFlags = sessionInfo.lastKeyFlags;
  latestFrameSummary.rightConnected = sessionInfo.rightConnected;
  latestFrameSummary.joyX = clampByte(payload.rightController?.joy?.x ?? 50);
  latestFrameSummary.joyY = clampByte(payload.rightController?.joy?.y ?? 50);
  latestFrameSummary.relPositionMm = toVec3(payload.rightController?.relPositionMm, latestFrameSummary.relPositionMm);
  latestFrameSummary.relQuaternionWxyz = normalizeQuatWxyz(
    toQuat(payload.rightController?.relQuaternionWxyz, latestFrameSummary.relQuaternionWxyz)
  );

  sessionCache.set(payload.sessionId, sessionInfo);
}

function relayPoseFrame(payload) {
  const config = loadRuntimeConfig();
  const right = payload?.rightController ?? null;
  if (!right?.connected) {
    return { ok: false, error: 'right_controller_unavailable' };
  }

  ensureBridgeConnection(config);
  const socket = bridgeState.socket;
  if (!socket || socket.destroyed) {
    return { ok: false, error: bridgeState.lastError || 'bridge_disconnected' };
  }

  const packet = buildUartPacket(payload, right, config);
  try {
    socket.write(packet);
    relayStats.framesRelayed += 1;
    relayStats.lastPacketBytes = packet.byteLength;
    relayStats.lastBridgeError = '';
    bridgeState.lastError = '';
    bridgeState.lastFrameSeq = Number(payload.frameSeq ?? 0) >>> 0;
    bridgeState.lastWriteAtMs = Date.now();
    appendTiming(relayTiming.bridgeTxTimesMs, bridgeState.lastWriteAtMs, 240);
    return { ok: true, error: '' };
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    relayStats.bridgeWriteErrors += 1;
    relayStats.lastBridgeError = message;
    bridgeState.lastError = message;
    return { ok: false, error: message };
  }
}

function buildUartPacket(payload, right, config) {
  const buffer = Buffer.alloc(XR_PACKET_SIZE);
  const seq = Number(payload?.frameSeq ?? 0) >>> 0;

  const absPosRaw = toVec3(right?.absPositionMm, [0, 0, 0]);
  const relPosRaw = toVec3(right?.relPositionMm, [0, 0, 0]);
  const absQuatRaw = normalizeQuatWxyz(toQuat(right?.absQuaternionWxyz, [1, 0, 0, 0]));
  const relQuatRaw = normalizeQuatWxyz(toQuat(right?.relQuaternionWxyz, [1, 0, 0, 0]));

  const positionScale = clampNumber(config.positionScale, 0.05, 8.0);
  const rotationScale = clampNumber(config.rotationScale, 0.05, 8.0);

  const absPosScaled = absPosRaw.map((value) => value * positionScale);
  const relPosScaled = relPosRaw.map((value) => value * positionScale);
  const absQuatScaled = scaleQuatFromIdentity(absQuatRaw, rotationScale);
  const relQuatScaled = scaleQuatFromIdentity(relQuatRaw, rotationScale);

  const absEuler = quatToEulerDeg(absQuatScaled);
  const relEuler = quatToEulerDeg(relQuatScaled);
  const joyX = clampByte(right?.joy?.x ?? 50);
  const joyY = clampByte(right?.joy?.y ?? 50);
  const keyFlags = clampU16(right?.keyFlags ?? 0);

  buffer.writeUInt32LE(XR_PACKET_MAGIC >>> 0, 0);
  buffer.writeUInt16LE(1, 4);
  buffer.writeUInt16LE(0x0001, 6);
  buffer.writeUInt32LE(seq, 8);
  buffer.writeUInt8(joyX, 12);
  buffer.writeUInt8(joyY, 13);
  buffer.writeUInt16LE(keyFlags, 14);
  writeVec3(buffer, 16, absPosScaled);
  writeVec3(buffer, 28, relPosScaled);
  writeQuat(buffer, 40, absQuatScaled);
  writeQuat(buffer, 56, relQuatScaled);
  writeVec3(buffer, 72, absEuler);
  writeVec3(buffer, 84, relEuler);
  return buffer;
}

function ensureBridgeConnection(config = loadRuntimeConfig()) {
  const host = String(config.bridgeHost ?? '127.0.0.1').trim() || '127.0.0.1';
  const port = clampPort(config.bridgePort);
  const targetChanged = bridgeState.host !== host || bridgeState.port !== port;

  if (targetChanged && bridgeState.socket) {
    bridgeState.socket.destroy();
  }
  if (targetChanged && bridgeState.reconnectTimer) {
    clearTimeout(bridgeState.reconnectTimer);
    bridgeState.reconnectTimer = null;
  }

  bridgeState.host = host;
  bridgeState.port = port;

  if (bridgeState.connecting) {
    return;
  }
  if (bridgeState.socket && !bridgeState.socket.destroyed) {
    return;
  }

  bridgeState.connecting = true;
  const socket = createConnection({ host, port }, () => {
    // 【修复点】：显式关闭 Nagle 算法，确保微小姿态帧立刻发往底层，绝不缓冲排队
    socket.setNoDelay(true);

    bridgeState.socket = socket;
    bridgeState.connecting = false;
    bridgeState.connectedAtMs = Date.now();
    bridgeState.lastError = '';
    console.log(`[bridge] connected ${host}:${port}`);
  });

  socket.on('error', (error) => {
    bridgeState.lastError = error.message;
    relayStats.lastBridgeError = error.message;
  });

  socket.on('close', () => {
    if (bridgeState.socket === socket) {
      bridgeState.socket = null;
    }
    bridgeState.connecting = false;
    bridgeState.lastDisconnectAtMs = Date.now();
    scheduleBridgeReconnect();
  });
}

function scheduleBridgeReconnect() {
  if (bridgeState.reconnectTimer) {
    return;
  }
  bridgeState.reconnectTimer = setTimeout(() => {
    bridgeState.reconnectTimer = null;
    ensureBridgeConnection();
  }, BRIDGE_RECONNECT_MS);
}

function buildRelayStatus(config) {
  const now = Date.now();
  return {
    bridgeHost: String(config.bridgeHost ?? '127.0.0.1').trim() || '127.0.0.1',
    bridgePort: clampPort(config.bridgePort),
    bridgeConnected: Boolean(bridgeState.socket && !bridgeState.socket.destroyed),
    bridgeAgeMs: bridgeState.lastWriteAtMs ? Math.max(0, now - bridgeState.lastWriteAtMs) : 0,
    bridgeLastSeq: bridgeState.lastFrameSeq,
    positionScale: clampNumber(config.positionScale, 0.05, 8.0),
    rotationScale: clampNumber(config.rotationScale, 0.05, 8.0),
    framesReceived: relayStats.framesReceived,
    framesRelayed: relayStats.framesRelayed,
    framesDroppedSeq: relayStats.framesDroppedSeq,
    bridgeWriteErrors: relayStats.bridgeWriteErrors,
    receiveRateHz: computeRateHz(relayTiming.questRxTimesMs),
    relayRateHz: computeRateHz(relayTiming.bridgeTxTimesMs),
    lastBridgeError: relayStats.lastBridgeError || bridgeState.lastError,
    lastPacketBytes: relayStats.lastPacketBytes,
    latestFrame: {
      sessionId: latestFrameSummary.sessionId,
      frameSeq: latestFrameSummary.frameSeq,
      questSentAtMs: latestFrameSummary.questSentAtMs,
      ageMs: latestFrameSummary.receivedAtMs ? Math.max(0, now - latestFrameSummary.receivedAtMs) : 0,
      relayAgeMs: latestFrameSummary.relayedAtMs ? Math.max(0, now - latestFrameSummary.relayedAtMs) : 0,
      questToPcMsEstimate:
        latestFrameSummary.questSentAtMs > 0
          ? Math.max(0, latestFrameSummary.receivedAtMs - latestFrameSummary.questSentAtMs)
          : 0,
      keyFlags: latestFrameSummary.keyFlags,
      rightConnected: latestFrameSummary.rightConnected,
      joyX: latestFrameSummary.joyX,
      joyY: latestFrameSummary.joyY,
      relPositionMm: latestFrameSummary.relPositionMm,
      relQuaternionWxyz: latestFrameSummary.relQuaternionWxyz
    }
  };
}

function serveStaticAsset(pathname, res) {
  const relativePath = pathname === '/' ? 'index.html' : pathname.slice(1);
  const assetPath = normalize(join(DIST_DIR, relativePath));

  if (!assetPath.startsWith(normalize(DIST_DIR))) {
    sendText(res, 403, 'Forbidden');
    return;
  }

  const candidate = existsSync(assetPath) && statSync(assetPath).isFile() ? assetPath : join(DIST_DIR, 'index.html');
  if (!existsSync(candidate)) {
    sendText(res, 404, 'dist not found. Run `npm run build` first.');
    return;
  }

  const mimeType = MIME_TYPES[extname(candidate)] ?? 'application/octet-stream';
  res.writeHead(200, { 'Content-Type': mimeType });
  res.end(readFileSync(candidate));
}

function sendJson(res, statusCode, payload) {
  res.writeHead(statusCode, { 'Content-Type': 'application/json; charset=utf-8' });
  res.end(JSON.stringify(payload, null, 2));
}

function sendText(res, statusCode, payload) {
  res.writeHead(statusCode, { 'Content-Type': 'text/plain; charset=utf-8' });
  res.end(payload);
}

function defaultRuntimeConfig() {
  return {
    bridgeHost: '127.0.0.1',
    bridgePort: 8791,
    positionScale: 1.0,
    rotationScale: 1.0,
    calibrationModeEnabled: false
  };
}

let cachedRuntimeConfig = defaultRuntimeConfig();
let cachedRuntimeConfigMtime = -1;

function loadRuntimeConfig() {
  try {
    if (!existsSync(RUNTIME_CONFIG_PATH)) {
      cachedRuntimeConfig = defaultRuntimeConfig();
      cachedRuntimeConfigMtime = -1;
      return cachedRuntimeConfig;
    }

    const stats = statSync(RUNTIME_CONFIG_PATH);
    if (stats.mtimeMs !== cachedRuntimeConfigMtime) {
      const raw = JSON.parse(readFileSync(RUNTIME_CONFIG_PATH, 'utf8'));
      cachedRuntimeConfig = {
        ...defaultRuntimeConfig(),
        ...(raw && typeof raw === 'object' ? raw : {})
      };
      cachedRuntimeConfigMtime = stats.mtimeMs;
      ensureBridgeConnection(cachedRuntimeConfig);
    }
  } catch (error) {
    console.error('[config] failed to load runtime-config.json', error);
  }

  return cachedRuntimeConfig;
}

function listAccessUrls(port, tlsActive) {
  const scheme = tlsActive ? 'https' : 'http';
  const urls = [`${scheme}://127.0.0.1:${port}`, `${scheme}://localhost:${port}`];
  const seen = new Set(['127.0.0.1', 'localhost']);
  const nets = networkInterfaces();

  for (const entries of Object.values(nets)) {
    for (const entry of entries ?? []) {
      if (!entry || entry.family !== 'IPv4' || entry.internal || seen.has(entry.address)) {
        continue;
      }
      seen.add(entry.address);
      urls.push(`${scheme}://${entry.address}:${port}`);
    }
  }

  return urls;
}

function appendTiming(buffer, valueMs, maxSize) {
  buffer.push(valueMs);
  if (buffer.length > maxSize) {
    buffer.splice(0, buffer.length - maxSize);
  }
}

function computeRateHz(values) {
  if (!Array.isArray(values) || values.length < 2) {
    return 0;
  }
  const dtMs = values[values.length - 1] - values[0];
  if (dtMs <= 0) {
    return 0;
  }
  return Number((((values.length - 1) * 1000) / dtMs).toFixed(2));
}

function writeVec3(buffer, offset, values) {
  buffer.writeFloatLE(Number(values[0] ?? 0), offset + 0);
  buffer.writeFloatLE(Number(values[1] ?? 0), offset + 4);
  buffer.writeFloatLE(Number(values[2] ?? 0), offset + 8);
}

function writeQuat(buffer, offset, values) {
  buffer.writeFloatLE(Number(values[0] ?? 1), offset + 0);
  buffer.writeFloatLE(Number(values[1] ?? 0), offset + 4);
  buffer.writeFloatLE(Number(values[2] ?? 0), offset + 8);
  buffer.writeFloatLE(Number(values[3] ?? 0), offset + 12);
}

function toVec3(value, fallback) {
  if (!Array.isArray(value) || value.length < 3) {
    return [...fallback];
  }
  return [Number(value[0] ?? fallback[0]), Number(value[1] ?? fallback[1]), Number(value[2] ?? fallback[2])];
}

function toQuat(value, fallback) {
  if (!Array.isArray(value) || value.length < 4) {
    return [...fallback];
  }
  return [
    Number(value[0] ?? fallback[0]),
    Number(value[1] ?? fallback[1]),
    Number(value[2] ?? fallback[2]),
    Number(value[3] ?? fallback[3])
  ];
}

function normalizeQuatWxyz(quat) {
  const [w, x, y, z] = quat;
  const length = Math.hypot(w, x, y, z);
  if (!Number.isFinite(length) || length < 1e-9) {
    return [1, 0, 0, 0];
  }
  return [w / length, x / length, y / length, z / length];
}

function multiplyQuatWxyz(a, b) {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw
  ];
}

function rotateVecByQuat(vector, quat) {
  const [vx, vy, vz] = vector;
  const [w, x, y, z] = normalizeQuatWxyz(quat);
  const ix = w * vx + y * vz - z * vy;
  const iy = w * vy + z * vx - x * vz;
  const iz = w * vz + x * vy - y * vx;
  const iw = -x * vx - y * vy - z * vz;
  return [
    ix * w + iw * -x + iy * -z - iz * -y,
    iy * w + iw * -y + iz * -x - ix * -z,
    iz * w + iw * -z + ix * -y - iy * -x
  ];
}

function addVec3(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scaleQuatFromIdentity(quat, scale) {
  const [w, x, y, z] = normalizeQuatWxyz(quat);
  const halfAngle = Math.acos(clampNumber(w, -1, 1));
  const sinHalf = Math.sin(halfAngle);
  if (sinHalf < 1e-6) {
    return [1, 0, 0, 0];
  }
  const axis = [x / sinHalf, y / sinHalf, z / sinHalf];
  const scaledHalf = halfAngle * scale;
  const scaledSin = Math.sin(scaledHalf);
  return normalizeQuatWxyz([
    Math.cos(scaledHalf),
    axis[0] * scaledSin,
    axis[1] * scaledSin,
    axis[2] * scaledSin
  ]);
}

function quatToEulerDeg(quat) {
  const [qw, qx, qy, qz] = normalizeQuatWxyz(quat);
  const sinrCosp = 2 * (qw * qx + qy * qz);
  const cosrCosp = 1 - 2 * (qx * qx + qy * qy);
  const roll = Math.atan2(sinrCosp, cosrCosp);
  const sinp = clampNumber(2 * (qw * qy - qz * qx), -1, 1);
  const pitch = Math.asin(sinp);
  const sinyCosp = 2 * (qw * qz + qx * qy);
  const cosyCosp = 1 - 2 * (qy * qy + qz * qz);
  const yaw = Math.atan2(sinyCosp, cosyCosp);
  const radToDeg = 180 / Math.PI;
  return [roll * radToDeg, pitch * radToDeg, yaw * radToDeg];
}

function clampNumber(value, min, max) {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) {
    return min;
  }
  return Math.min(max, Math.max(min, numeric));
}

function clampPort(value) {
  return Math.max(1, Math.min(65535, Math.round(Number(value) || 8791)));
}

function clampByte(value) {
  return Math.max(0, Math.min(100, Math.round(Number(value) || 0)));
}

function clampU16(value) {
  return Math.max(0, Math.min(0xffff, Math.round(Number(value) || 0)));
}

const MIME_TYPES = {
  '.css': 'text/css; charset=utf-8',
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.woff2': 'font/woff2'
};
