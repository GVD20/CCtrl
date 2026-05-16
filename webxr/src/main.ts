import './style.css';

import * as THREE from 'three';

type Handedness = 'left' | 'right';
type SessionMode = 'desktop' | 'immersive-ar' | 'immersive-vr';

type Vec3Tuple = [number, number, number];
type QuatWxyzTuple = [number, number, number, number];

type ControllerFrameSample = {
  connected: boolean;
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
  rawQuaternion: THREE.Quaternion;
  axes: number[];
  buttonsRaw: number[];
  triggerValue: number;
  squeezeValue: number;
  thumbstickPressed: boolean;
  primaryPressed: boolean;
  secondaryPressed: boolean;
};

type ControllerState = {
  connected: boolean;
  lastPoseSeenMs: number;
  rawPositionMm: Vec3Tuple;
  relPositionMm: Vec3Tuple;
  txPositionMm: Vec3Tuple;
  rawQuaternionWxyz: QuatWxyzTuple;
  relQuaternionWxyz: QuatWxyzTuple;
  txQuaternionWxyz: QuatWxyzTuple;
  axes: number[];
  buttonsRaw: number[];
  triggerValue: number;
  squeezeValue: number;
  thumbstickPressed: boolean;
  primaryPressed: boolean;
  secondaryPressed: boolean;
  joyX: number;
  joyY: number;
};

type WireButtons = {
  trigger: number;
  squeeze: number;
  thumbstick: boolean;
  a: boolean;
  b: boolean;
};

type WireController = {
  connected: boolean;
  absPositionMm: Vec3Tuple;
  relPositionMm: Vec3Tuple;
  absQuaternionWxyz: QuatWxyzTuple;
  relQuaternionWxyz: QuatWxyzTuple;
  joy: { x: number; y: number };
  buttons: WireButtons;
  keyFlags: number;
  panelState: {
    debug: boolean;
    keys: boolean;
    info: boolean;
  };
};

type PoseFrameMessage = {
  type: 'pose_frame';
  sessionId: string;
  frameSeq: number;
  xrTimestampMs: number;
  sentAtMs: number;
  referenceSpace: 'local';
  sessionMode: Exclude<SessionMode, 'desktop'>;
  targetHand: 'right';
  controlFrame: {
    positionMm: Vec3Tuple;
    quaternionWxyz: QuatWxyzTuple;
    visible: boolean;
    fade: number;
  };
  leftController: WireController;
  rightController: WireController;
};

type ControllerVisual = {
  root: THREE.Group;
  header: THREE.Sprite;
  directionArrowMaterial: THREE.MeshStandardMaterial;
  directionArrowIdleColor: THREE.Color;
  directionArrowPressedColor: THREE.Color;
};

type HandGestureFingerName = 'thumb' | 'index' | 'middle' | 'ring' | 'pinky';
type HandJointName =
  | 'wrist'
  | 'thumb-tip'
  | 'index-finger-tip'
  | 'middle-finger-tip'
  | 'ring-finger-tip'
  | 'pinky-finger-tip'
  | 'index-finger-metacarpal'
  | 'middle-finger-phalanx-proximal'
  | 'ring-finger-phalanx-proximal'
  | 'pinky-finger-metacarpal'
  | 'pinky-finger-phalanx-proximal';

type HandFrameSample = {
  connected: boolean;
  wrist: THREE.Vector3;
  fingertips: Record<HandGestureFingerName, THREE.Vector3>;
  palmUp: boolean;
  openHand: boolean;
  stayEligible: boolean;
  touchIndex: boolean;
  touchMiddle: boolean;
  touchRing: boolean;
};

type HandGestureVisual = {
  root: THREE.Group;
  header: THREE.Sprite;
  markers: Record<HandGestureFingerName, THREE.Mesh<THREE.SphereGeometry, THREE.MeshStandardMaterial>>;
  keyLabels: {
    middle: THREE.Sprite;
    ring: THREE.Sprite;
  };
};

const HAND_GESTURE_FINGERS = ['thumb', 'index', 'middle', 'ring', 'pinky'] as const satisfies readonly HandGestureFingerName[];

type PanelButton = {
  id: number;
  label: string;
  subLabel: string;
  bitMask: number;
  rect: { x: number; y: number; width: number; height: number };
};

type WorldPanel = {
  key: 'debug' | 'keys' | 'info';
  root: THREE.Group;
  mesh: THREE.Mesh<THREE.PlaneGeometry, THREE.MeshBasicMaterial>;
  canvas: HTMLCanvasElement;
  texture: THREE.CanvasTexture;
  width: number;
  height: number;
  accent: string;
  title: string;
  visible: boolean;
  buttons: PanelButton[];
};

type RuntimeConfigView = {
  positionScale: number;
  rotationScale: number;
  calibrationModeEnabled: boolean;
  lastFetchedAtMs: number;
};

declare global {
  interface Window {
    __WEBXR_CONFIG__?: {
      WS_URL?: string;
      SEND_HZ?: number;
    };
  }
}

class PoseTransport {
  private socket: WebSocket | null = null;
  private reconnectTimer: number | null = null;
  statusLabel = '正在连接';

  constructor(private readonly targetUrl: string) { }

  connect() {
    if (this.socket && (this.socket.readyState === WebSocket.OPEN || this.socket.readyState === WebSocket.CONNECTING)) {
      return;
    }

    this.statusLabel = '正在连接';
    const socket = new WebSocket(this.targetUrl);
    this.socket = socket;

    socket.addEventListener('open', () => {
      this.statusLabel = '已连接';
      this.send({
        type: 'hello',
        protocolVersion: 2,
        userAgent: navigator.userAgent,
        xrSupported: Boolean(navigator.xr)
      });
    });

    socket.addEventListener('close', () => {
      this.statusLabel = '已断开';
      this.scheduleReconnect();
    });

    socket.addEventListener('error', () => {
      this.statusLabel = '连接异常';
    });
  }

  send(payload: unknown) {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
      return;
    }

    this.socket.send(JSON.stringify(payload));
  }

  private scheduleReconnect() {
    if (this.reconnectTimer !== null) {
      return;
    }

    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, 1500);
  }
}

const DEFAULT_SEND_HZ = 60;
const REFERENCE_SPACE_TYPE = 'local' as const;
const KEY_PANEL_BITS = [0x01, 0x02, 0x04, 0x08] as const;
const DELTA_KEY_BIT = 0x10;
const KEY6_BIT = 0x20;
const KEY7_BIT = 0x40;
const TRIGGER_PRESS_THRESHOLD = 0.72;
const PANEL_HOLD_MS = 450;
const AXIS_FADE_MS = 5000;
const CONTROLLER_POSE_HOLD_MS = 140;
const AUTO_ENTER_XR_ON_LOAD = true;
const ENABLE_HAND_GESTURE_KEYS = true;
const HAND_GESTURE_ENTER_HOLD_MS = 300;
const HAND_GESTURE_EXIT_HOLD_MS = 260;
const HAND_GESTURE_TOUCH_DISTANCE_M = 0.035;
const HAND_GESTURE_PALM_UP_DOT = 0.55;
const HAND_GESTURE_STAY_PALM_UP_DOT = 0.32;
const HAND_GESTURE_FINGER_EXTEND_MARGIN_M = 0.03;
const HAND_GESTURE_STAY_FINGER_EXTEND_MARGIN_M = 0.016;
const HAND_GESTURE_MIN_SPACING_M = 0.024;
const HAND_GESTURE_STAY_MIN_SPACING_M = 0.012;
const HAND_GESTURE_THUMB_INDEX_SPACING_M = 0.04;
const HAND_GESTURE_STAY_THUMB_INDEX_SPACING_M = 0.022;
const HAND_GESTURE_STILL_DELTA_M = 0.006;
const HAND_GESTURE_ACTIVE_MARKER_SCALE = 1.45;
const HAND_GESTURE_IDLE_MARKER_SCALE = 1;

const pageConfig = window.__WEBXR_CONFIG__ ?? {};
const wsUrl = resolveWsUrl(pageConfig.WS_URL);
const sendHz = Number.isFinite(pageConfig.SEND_HZ) ? Math.max(1, Number(pageConfig.SEND_HZ)) : DEFAULT_SEND_HZ;

const xrStatusEl = mustElement<HTMLDivElement>('xr-status');
const socketStatusEl = mustElement<HTMLDivElement>('socket-status');
const modeStatusEl = mustElement<HTMLDivElement>('mode-status');
const sendHzStatusEl = mustElement<HTMLDivElement>('send-hz-status');
const lastFrameEl = mustElement<HTMLPreElement>('last-frame');
const xrButtonEl = mustElement<HTMLButtonElement>('xr-enter');
sendHzStatusEl.textContent = `${sendHz}`;

const app = document.getElementById('app');
if (!app) {
  throw new Error('App root not found');
}

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType(REFERENCE_SPACE_TYPE);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
app.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const desktopBackground = new THREE.Color('#07131f');
const desktopFog = new THREE.Fog('#07131f', 2, 12);
scene.background = desktopBackground;
scene.fog = desktopFog;

const camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 0.01, 50);
camera.position.set(0, 1.45, 2.4);

scene.add(new THREE.HemisphereLight(0xd6eeff, 0x132132, 1.3));
const keyLight = new THREE.DirectionalLight(0xffffff, 1.15);
keyLight.position.set(2, 4, 2.5);
scene.add(keyLight);

const transport = new PoseTransport(wsUrl);
const tempMatrix = new THREE.Matrix4();
const tempPosition = new THREE.Vector3();
const tempQuaternion = new THREE.Quaternion();
const tempScale = new THREE.Vector3();
const scratchVecA = new THREE.Vector3();
const scratchVecB = new THREE.Vector3();
const scratchVecC = new THREE.Vector3();
const scratchQuatA = new THREE.Quaternion();
const scratchQuatB = new THREE.Quaternion();
const scratchQuatC = new THREE.Quaternion();
const scratchHead = new THREE.Vector3();
const raycaster = new THREE.Raycaster();
const worldUp = new THREE.Vector3(0, 1, 0);
// World/position contract redefined on top of native WebXR space:
//   +X = native +X
//   +Y = native +Z  (mirrored from the old forward direction)
//   +Z = native +Y
// This matches the ESP32 ABS/world frame as a left-handed Z-up system.
const txWorldAxisX = new THREE.Vector3(1, 0, 0);
const txWorldAxisY = new THREE.Vector3(0, 0, 1);
const txWorldAxisZ = new THREE.Vector3(0, 1, 0);
// Handle/local attitude contract stays on the agreed IMU-like axis definition:
//   +X = native +X
//   +Y = native -Z
//   +Z = native +Y
const txLocalAxisX = new THREE.Vector3(1, 0, 0);
const txLocalAxisY = new THREE.Vector3(0, 0, -1);
const txLocalAxisZ = new THREE.Vector3(0, 1, 0);
// Apply a fixed local roll for the hand/controller local frame only. World
// frame calibration must keep using the raw WebXR grip quaternion.
const controllerGripPoseOffset = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);

const currentSessionMode = { value: 'desktop' as SessionMode };
const xrSupport = { ar: false, vr: false };
const runtimeConfigView: RuntimeConfigView = {
  positionScale: 1,
  rotationScale: 1,
  calibrationModeEnabled: false,
  lastFetchedAtMs: 0
};

let activeReferenceSpace: XRReferenceSpace | null = null;
let referenceSpaceRequest: Promise<XRReferenceSpace> | null = null;
let xrSessionId: string | null = null;
let xrFrameSeq = 0;
let pendingSessionMode: Exclude<SessionMode, 'desktop'> | null = null;
let xrAutoEnterAttempted = false;

const controlFrame = {
  initialized: false,
  position: new THREE.Vector3(),
  quaternion: new THREE.Quaternion(),
  adjustStartPosition: new THREE.Vector3(),
  adjustStartQuaternion: new THREE.Quaternion(),
  hasAdjustReference: false,
  adjustSource: null as 'right-controller' | 'left-hand' | null,
  fadeAlpha: 0,
  lastReleaseMs: 0,
  active: false
};

const originVisualRoot = buildControlFrameVisual();
scene.add(originVisualRoot);
primeMaterialOpacity(originVisualRoot);

const desktopRig = new THREE.Group();
desktopRig.position.set(0, 1.35, -0.72);
desktopRig.add(createFloatingTag('桌面预览', '#38bdf8'));
scene.add(desktopRig);

const controllerVisuals = {
  left: createControllerVisual('left'),
  right: createControllerVisual('right')
} satisfies Record<Handedness, ControllerVisual>;
scene.add(controllerVisuals.left.root, controllerVisuals.right.root);

const handGestureVisual = createHandGestureVisual();
scene.add(handGestureVisual.root);

const panels = {
  debug: createWorldPanel('debug', '调试面板', '#f59e0b', 1024, 640),
  keys: createWorldPanel('keys', '按键面板', '#22c55e', 960, 620, [
    { id: 4, label: 'KEY 4', subLabel: 'KEY 4', bitMask: KEY_PANEL_BITS[3], rect: { x: 72, y: 184, width: 232, height: 138 } },
    { id: 3, label: 'KEY 3', subLabel: 'KEY 3', bitMask: KEY_PANEL_BITS[2], rect: { x: 364, y: 184, width: 232, height: 138 } },
    { id: 2, label: 'KEY 2', subLabel: 'KEY 2', bitMask: KEY_PANEL_BITS[1], rect: { x: 656, y: 184, width: 232, height: 138 } },
    { id: 1, label: 'KEY 1', subLabel: 'KEY 1', bitMask: KEY_PANEL_BITS[0], rect: { x: 72, y: 364, width: 232, height: 138 } },
    { id: 6, label: 'KEY 6', subLabel: 'KEY 6', bitMask: KEY6_BIT, rect: { x: 364, y: 364, width: 232, height: 138 } },
    { id: 7, label: 'KEY 7', subLabel: 'KEY 7', bitMask: KEY7_BIT, rect: { x: 656, y: 364, width: 232, height: 138 } }
  ]),
  info: createWorldPanel('info', '状态面板', '#38bdf8', 1080, 700)
};
scene.add(panels.debug.root, panels.keys.root, panels.info.root);

const controllerState = {
  left: createEmptyState(),
  right: createEmptyState()
} satisfies Record<Handedness, ControllerState>;

const buttonHistory = {
  right: {
    primary: false,
    secondary: false,
    primaryStartedAtMs: 0,
    secondaryStartedAtMs: 0,
    primaryLongHoldActive: false,
    secondaryLongHoldActive: false
  }
};

const panelInteraction = {
  hoveredKeyButtonId: 0,
  activeKeyButtonId: 0,
  keyFlags: 0
};

const handGestureState = {
  featureEnabled: ENABLE_HAND_GESTURE_KEYS,
  handTracked: false,
  palmUp: false,
  openHand: false,
  still: false,
  eligible: false,
  active: false,
  eligibleSinceMs: 0,
  ineligibleSinceMs: 0,
  touchMiddle: false,
  touchRing: false,
  keyFlags: 0,
  cooldownUntilMs: 0,
  cooldownActive: false,
  previousRawTouchMiddle: false,
  previousRawTouchRing: false,
  hasPreviousPose: false,
  previousWrist: new THREE.Vector3(),
  previousFingertips: {
    thumb: new THREE.Vector3(),
    index: new THREE.Vector3(),
    middle: new THREE.Vector3(),
    ring: new THREE.Vector3(),
    pinky: new THREE.Vector3()
  } satisfies Record<HandGestureFingerName, THREE.Vector3>
};

applySessionVisuals('desktop');
updateControlFrameVisual(0);
transport.connect();
void setupXrButton();
void refreshRuntimeConfig();

renderer.xr.addEventListener('sessionstart', () => {
  const mode = pendingSessionMode ?? (xrSupport.ar ? 'immersive-ar' : 'immersive-vr');
  currentSessionMode.value = mode;
  activeReferenceSpace = null;
  referenceSpaceRequest = null;
  xrSessionId = crypto.randomUUID();
  xrFrameSeq = 0;
  resetControlFrame();
  resetPanels();
  resetControllers();
  applySessionVisuals(mode);
});

renderer.xr.addEventListener('sessionend', () => {
  transport.send({
    type: 'session_end',
    sessionId: xrSessionId,
    endedAtMs: Date.now(),
    reason: 'xr-session-ended'
  });
  xrSessionId = null;
  activeReferenceSpace = null;
  referenceSpaceRequest = null;
  pendingSessionMode = null;
  currentSessionMode.value = 'desktop';
  resetControlFrame();
  resetControllers();
  applySessionVisuals('desktop');
});

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

renderer.setAnimationLoop((time, frame) => {
  if (renderer.xr.isPresenting && frame) {
    updateXrFrame(frame);
  } else {
    updateDesktopPreview(time);
  }

  updateUiSummary();
  renderer.render(scene, camera);
});

async function setupXrButton() {
  if (!navigator.xr) {
    xrStatusEl.textContent = '当前浏览器不支持 WebXR';
    xrButtonEl.textContent = 'XR 不可用';
    xrButtonEl.disabled = true;
    return;
  }

  xrButtonEl.disabled = true;
  xrButtonEl.addEventListener('click', () => {
    void toggleXrSession();
  });

  try {
    const [arSupported, vrSupported] = await Promise.all([
      navigator.xr.isSessionSupported('immersive-ar').catch(() => false),
      navigator.xr.isSessionSupported('immersive-vr').catch(() => false)
    ]);

    xrSupport.ar = arSupported;
    xrSupport.vr = vrSupported;
    xrStatusEl.textContent = arSupported ? '混合现实已就绪' : vrSupported ? '仅支持沉浸式 VR' : '无沉浸式支持';
    xrButtonEl.textContent = arSupported ? '进入 XR' : vrSupported ? '进入 VR' : 'XR 不可用';
    xrButtonEl.disabled = !(arSupported || vrSupported);
    scheduleAutoEnterXr();
  } catch {
    xrStatusEl.textContent = 'XR 能力检测失败';
    xrButtonEl.textContent = 'XR 不可用';
    xrButtonEl.disabled = true;
  }
}

function scheduleAutoEnterXr() {
  if (!AUTO_ENTER_XR_ON_LOAD || xrAutoEnterAttempted || renderer.xr.isPresenting || !(xrSupport.ar || xrSupport.vr)) {
    return;
  }

  const attempt = () => {
    if (xrAutoEnterAttempted || renderer.xr.isPresenting) {
      return;
    }
    xrAutoEnterAttempted = true;
    void toggleXrSession();
  };

  if (document.visibilityState === 'visible') {
    window.setTimeout(attempt, 80);
    return;
  }

  const onVisible = () => {
    if (document.visibilityState !== 'visible') {
      return;
    }
    document.removeEventListener('visibilitychange', onVisible);
    window.setTimeout(attempt, 80);
  };
  document.addEventListener('visibilitychange', onVisible);
}

async function toggleXrSession() {
  const activeSession = renderer.xr.getSession();
  if (activeSession) {
    await activeSession.end();
    return;
  }

  xrButtonEl.disabled = true;
  xrStatusEl.textContent = xrSupport.ar ? '正在启动 XR…' : '正在启动 VR…';

  try {
    if (xrSupport.ar) {
      await startSession('immersive-ar');
      return;
    }

    if (xrSupport.vr) {
      await startSession('immersive-vr');
      return;
    }
  } catch (error) {
    if (xrSupport.ar && xrSupport.vr) {
      try {
        xrStatusEl.textContent = 'MR 启动失败，尝试退回 VR…';
        await startSession('immersive-vr');
        return;
      } catch (fallbackError) {
        xrStatusEl.textContent = `启动失败：${String(fallbackError)}`;
      }
    } else {
      xrStatusEl.textContent = `启动失败：${String(error)}`;
    }
  } finally {
    if (!renderer.xr.isPresenting) {
      xrButtonEl.disabled = false;
      xrButtonEl.textContent = xrSupport.ar ? '进入 XR' : xrSupport.vr ? '进入 VR' : 'XR 不可用';
    }
  }
}

async function startSession(mode: Exclude<SessionMode, 'desktop'>) {
  const sessionInit: XRSessionInit = {
    optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking' as never]
  };

  pendingSessionMode = mode;
  const session = await navigator.xr!.requestSession(mode, sessionInit);
  await renderer.xr.setSession(session);
}

function updateXrFrame(frame: XRFrame) {
  const session = frame.session;
  const nowMs = Date.now();

  if (!activeReferenceSpace) {
    if (!referenceSpaceRequest) {
      referenceSpaceRequest = session.requestReferenceSpace(REFERENCE_SPACE_TYPE);
      referenceSpaceRequest
        .then((referenceSpace) => {
          activeReferenceSpace = referenceSpace;
          transport.send({
            type: 'session_start',
            sessionId: xrSessionId,
            startedAtMs: Date.now(),
            referenceSpace: REFERENCE_SPACE_TYPE,
            sessionMode: currentSessionMode.value,
            userAgent: navigator.userAgent
          });
        })
        .catch((error) => {
          xrStatusEl.textContent = `参考空间初始化失败：${String(error)}`;
        });
    }
    return;
  }

  const samples = {
    left: null,
    right: null
  } as Record<Handedness, ControllerFrameSample | null>;
  const handSamples = {
    left: null,
    right: null
  } as Record<Handedness, HandFrameSample | null>;

  for (const inputSource of session.inputSources) {
    if (inputSource.handedness !== 'left' && inputSource.handedness !== 'right') {
      continue;
    }

    if (ENABLE_HAND_GESTURE_KEYS) {
      handSamples[inputSource.handedness] = createHandFrameSample(frame, inputSource, activeReferenceSpace);
    }

    const space = inputSource.gripSpace ?? inputSource.targetRaySpace;
    const pose = frame.getPose(space, activeReferenceSpace);
    if (!pose) {
      continue;
    }

    samples[inputSource.handedness] = createFrameSample(pose, inputSource.gamepad ?? null);
  }

  updateControlFrameFromInput(samples.right, samples.left, handSamples.left);
  applyControllerSamples(samples, frame.predictedDisplayTime);
  handlePanelInput(samples.right);
  updateHandGestureState(handSamples.right, nowMs);
  panelInteraction.keyFlags |= handGestureState.keyFlags;
  drawPanels();
  updateControlFrameVisual(nowMs);

  xrFrameSeq += 1;
  const message: PoseFrameMessage = {
    type: 'pose_frame',
    sessionId: xrSessionId ?? 'pending-session',
    frameSeq: xrFrameSeq,
    xrTimestampMs: frame.predictedDisplayTime,
    sentAtMs: Date.now(),
    referenceSpace: REFERENCE_SPACE_TYPE,
    sessionMode: currentSessionMode.value === 'desktop' ? 'immersive-vr' : currentSessionMode.value,
    targetHand: 'right',
    controlFrame: {
      positionMm: vectorMetersToMmTuple(controlFrame.position),
      quaternionWxyz: threeQuatToWxyzTuple(controlFrame.quaternion),
      visible: controlFrame.fadeAlpha > 0.01,
      fade: round3(controlFrame.fadeAlpha)
    },
    leftController: buildWireController('left'),
    rightController: buildWireController('right')
  };
  transport.send(message);
}

function createFrameSample(pose: XRPose, gamepad: Gamepad | null): ControllerFrameSample {
  tempMatrix.fromArray(pose.transform.matrix);
  tempMatrix.decompose(tempPosition, tempQuaternion, tempScale);
  const rawQuaternion = tempQuaternion.clone();
  tempQuaternion.multiply(controllerGripPoseOffset).normalize();

  const axes = gamepad ? [...gamepad.axes] : [];
  return {
    connected: true,
    position: tempPosition.clone(),
    quaternion: tempQuaternion.clone(),
    rawQuaternion,
    axes,
    buttonsRaw: gamepad ? gamepad.buttons.map((button) => round3(button.value)) : [],
    triggerValue: gamepad ? round3(gamepad.buttons[0]?.value ?? 0) : 0,
    squeezeValue: gamepad ? round3(gamepad.buttons[1]?.value ?? 0) : 0,
    thumbstickPressed: gamepad ? Boolean(gamepad.buttons[3]?.pressed) : false,
    primaryPressed: gamepad ? Boolean(gamepad.buttons[4]?.pressed) : false,
    secondaryPressed: gamepad ? Boolean(gamepad.buttons[5]?.pressed) : false
  };
}

function createHandFrameSample(frame: XRFrame, inputSource: XRInputSource, referenceSpace: XRReferenceSpace): HandFrameSample | null {
  const hand = getInputSourceHand(inputSource);
  if (!hand) {
    return null;
  }

  const wrist = readHandJointPosition(frame, hand, 'wrist', referenceSpace);
  const thumbTip = readHandJointPosition(frame, hand, 'thumb-tip', referenceSpace);
  const indexTip = readHandJointPosition(frame, hand, 'index-finger-tip', referenceSpace);
  const middleTip = readHandJointPosition(frame, hand, 'middle-finger-tip', referenceSpace);
  const ringTip = readHandJointPosition(frame, hand, 'ring-finger-tip', referenceSpace);
  const pinkyTip = readHandJointPosition(frame, hand, 'pinky-finger-tip', referenceSpace);
  const indexMetacarpal = readHandJointPosition(frame, hand, 'index-finger-metacarpal', referenceSpace);
  const middleProximal = readHandJointPosition(frame, hand, 'middle-finger-phalanx-proximal', referenceSpace);
  const ringProximal = readHandJointPosition(frame, hand, 'ring-finger-phalanx-proximal', referenceSpace);
  const pinkyMetacarpal = readHandJointPosition(frame, hand, 'pinky-finger-metacarpal', referenceSpace);
  const pinkyProximal = readHandJointPosition(frame, hand, 'pinky-finger-phalanx-proximal', referenceSpace);

  if (!wrist || !thumbTip || !indexTip || !middleTip || !ringTip || !pinkyTip || !indexMetacarpal || !middleProximal || !ringProximal || !pinkyMetacarpal || !pinkyProximal) {
    return null;
  }

  const fingertips = {
    thumb: thumbTip,
    index: indexTip,
    middle: middleTip,
    ring: ringTip,
    pinky: pinkyTip
  } satisfies Record<HandGestureFingerName, THREE.Vector3>;

  const palmAcross = scratchVecA.copy(indexMetacarpal).sub(pinkyMetacarpal);
  const fingerForward = scratchVecB.copy(middleTip).sub(wrist);
  const palmNormal = scratchVecC.copy(palmAcross).cross(fingerForward).normalize();
  const palmUp = palmNormal.dot(new THREE.Vector3(0, 1, 0)) >= HAND_GESTURE_PALM_UP_DOT;
  const stayPalmUp = palmNormal.dot(new THREE.Vector3(0, 1, 0)) >= HAND_GESTURE_STAY_PALM_UP_DOT;

  const thumbExtended = isFingerExtended(wrist, thumbTip, indexMetacarpal);
  const thumbStayExtended = isFingerExtended(wrist, thumbTip, indexMetacarpal, HAND_GESTURE_STAY_FINGER_EXTEND_MARGIN_M);
  const indexExtended = isFingerExtended(wrist, indexTip, indexMetacarpal);
  const indexStayExtended = isFingerExtended(wrist, indexTip, indexMetacarpal, HAND_GESTURE_STAY_FINGER_EXTEND_MARGIN_M);
  const middleExtended = isFingerExtended(wrist, middleTip, middleProximal);
  const middleStayExtended = isFingerExtended(wrist, middleTip, middleProximal, HAND_GESTURE_STAY_FINGER_EXTEND_MARGIN_M);
  const ringExtended = isFingerExtended(wrist, ringTip, ringProximal);
  const ringStayExtended = isFingerExtended(wrist, ringTip, ringProximal, HAND_GESTURE_STAY_FINGER_EXTEND_MARGIN_M);
  const pinkyExtended = isFingerExtended(wrist, pinkyTip, pinkyProximal);
  const pinkyStayExtended = isFingerExtended(wrist, pinkyTip, pinkyProximal, HAND_GESTURE_STAY_FINGER_EXTEND_MARGIN_M);
  const spacingOpen =
    thumbTip.distanceTo(indexTip) >= HAND_GESTURE_THUMB_INDEX_SPACING_M &&
    indexTip.distanceTo(middleTip) >= HAND_GESTURE_MIN_SPACING_M &&
    middleTip.distanceTo(ringTip) >= HAND_GESTURE_MIN_SPACING_M &&
    ringTip.distanceTo(pinkyTip) >= HAND_GESTURE_MIN_SPACING_M;
  const spacingStayOpen =
    thumbTip.distanceTo(indexTip) >= HAND_GESTURE_STAY_THUMB_INDEX_SPACING_M &&
    indexTip.distanceTo(middleTip) >= HAND_GESTURE_STAY_MIN_SPACING_M &&
    middleTip.distanceTo(ringTip) >= HAND_GESTURE_STAY_MIN_SPACING_M &&
    ringTip.distanceTo(pinkyTip) >= HAND_GESTURE_STAY_MIN_SPACING_M;
  const openHand = thumbExtended && indexExtended && middleExtended && ringExtended && pinkyExtended && spacingOpen;
  const stayEligible = stayPalmUp && thumbStayExtended && indexStayExtended && middleStayExtended && ringStayExtended && pinkyStayExtended && spacingStayOpen;

  return {
    connected: true,
    wrist,
    fingertips,
    palmUp,
    openHand,
    stayEligible,
    touchIndex: thumbTip.distanceTo(indexTip) <= HAND_GESTURE_TOUCH_DISTANCE_M,
    touchMiddle: thumbTip.distanceTo(middleTip) <= HAND_GESTURE_TOUCH_DISTANCE_M,
    touchRing: thumbTip.distanceTo(ringTip) <= HAND_GESTURE_TOUCH_DISTANCE_M
  };
}

function applyControllerSamples(samples: Record<Handedness, ControllerFrameSample | null>, nowMs: number) {
  for (const handedness of ['left', 'right'] as const) {
    const sample = samples[handedness];
    if (!sample) {
      const state = controllerState[handedness];
      if (!state.connected || nowMs - state.lastPoseSeenMs > CONTROLLER_POSE_HOLD_MS) {
        setControllerDisconnected(handedness);
      }
      continue;
    }

    const relPosition = sample.position.clone().sub(controlFrame.position);
    if (controlFrame.initialized) {
      relPosition.applyQuaternion(scratchQuatA.copy(controlFrame.quaternion).conjugate());
      scratchQuatB.copy(controlFrame.quaternion).conjugate().multiply(sample.quaternion);
    } else {
      scratchQuatB.copy(sample.quaternion);
    }

    const state = controllerState[handedness];
    state.connected = true;
    state.lastPoseSeenMs = nowMs;
    state.rawPositionMm = vectorMetersToMmTuple(sample.position);
    state.relPositionMm = vectorMetersToMmTuple(relPosition);
    state.txPositionMm = [...state.relPositionMm];
    state.rawQuaternionWxyz = threeQuatToWxyzTuple(sample.quaternion);
    state.relQuaternionWxyz = threeQuatToWxyzTuple(scratchQuatB);
    state.txQuaternionWxyz = [...state.relQuaternionWxyz];
    state.axes = [...sample.axes];
    state.buttonsRaw = [...sample.buttonsRaw];
    state.triggerValue = sample.triggerValue;
    state.squeezeValue = sample.squeezeValue;
    state.thumbstickPressed = sample.thumbstickPressed;
    state.primaryPressed = sample.primaryPressed;
    state.secondaryPressed = sample.secondaryPressed;
    state.joyX = mapAxisToPercent(resolveStickAxis(sample.axes, 0));
    state.joyY = mapAxisToPercent(-resolveStickAxis(sample.axes, 1));

    updateControllerVisual(handedness, sample);
  }
}

function updateHandGestureState(sample: HandFrameSample | null, nowMs: number) {
  if (!ENABLE_HAND_GESTURE_KEYS || !sample?.connected) {
    handGestureState.handTracked = false;
    handGestureState.palmUp = false;
    handGestureState.openHand = false;
    handGestureState.still = false;
    handGestureState.eligible = false;
    handGestureState.active = false;
    handGestureState.eligibleSinceMs = 0;
    handGestureState.ineligibleSinceMs = 0;
    handGestureState.touchMiddle = false;
    handGestureState.touchRing = false;
    handGestureState.keyFlags = 0;
    handGestureState.hasPreviousPose = false;
    updateHandGestureVisual(null);
    return;
  }

  handGestureState.handTracked = true;
  handGestureState.palmUp = sample.palmUp;
  handGestureState.openHand = sample.openHand;
  handGestureState.still = isHandPoseStill(sample);
  const entryEligible = sample.palmUp && sample.openHand && handGestureState.still;
  const stayEligible = sample.stayEligible;
  handGestureState.eligible = handGestureState.active ? stayEligible : entryEligible;
  rememberHandPose(sample);

  if (handGestureState.eligible) {
    handGestureState.ineligibleSinceMs = 0;
    if (handGestureState.eligibleSinceMs === 0) {
      handGestureState.eligibleSinceMs = nowMs;
    }
    if (!handGestureState.active && nowMs - handGestureState.eligibleSinceMs >= HAND_GESTURE_ENTER_HOLD_MS) {
      handGestureState.active = true;
    }
  } else {
    handGestureState.eligibleSinceMs = 0;
    if (handGestureState.ineligibleSinceMs === 0) {
      handGestureState.ineligibleSinceMs = nowMs;
    }
    if (handGestureState.active && nowMs - handGestureState.ineligibleSinceMs >= HAND_GESTURE_EXIT_HOLD_MS) {
      handGestureState.active = false;
    }
  }

  handGestureState.touchMiddle = handGestureState.active && sample.touchMiddle;
  handGestureState.touchRing = handGestureState.active && sample.touchRing;
  handGestureState.keyFlags = 0;
  if (handGestureState.touchMiddle) {
    handGestureState.keyFlags |= KEY6_BIT;
  }
  if (handGestureState.touchRing) {
    handGestureState.keyFlags |= KEY7_BIT;
  }

  updateHandGestureVisual(handGestureState.active ? sample : null);
  if (handGestureState.active) {
    controllerVisuals.left.root.visible = false;
    controllerVisuals.right.root.visible = false;
  }
}

function updateControlFrameFromInput(
  rightSample: ControllerFrameSample | null,
  leftSample: ControllerFrameSample | null,
  leftHandSample: HandFrameSample | null
) {
  const anchorSample = rightSample?.connected ? rightSample : leftSample?.connected ? leftSample : null;
  if (!anchorSample?.connected) {
    controlFrame.active = false;
    controlFrame.hasAdjustReference = false;
    controlFrame.adjustSource = null;
    return;
  }

  if (!controlFrame.initialized) {
    controlFrame.position.copy(anchorSample.position);
    setYawQuaternion(controlFrame.quaternion, extractYawRadians(anchorSample.rawQuaternion));
    controlFrame.initialized = true;
    controlFrame.fadeAlpha = 0;
  }

  const useRightControllerCalibration = Boolean(rightSample?.connected && rightSample.thumbstickPressed);
  const useLeftHandCalibration =
    !useRightControllerCalibration &&
    runtimeConfigView.calibrationModeEnabled &&
    Boolean(leftSample?.connected && leftHandSample?.touchIndex);
  const calibrationSample = useRightControllerCalibration ? rightSample : useLeftHandCalibration ? leftSample : null;
  const calibrationSource = useRightControllerCalibration ? 'right-controller' : useLeftHandCalibration ? 'left-hand' : null;

  if (calibrationSample && calibrationSource) {
    if (!controlFrame.hasAdjustReference || controlFrame.adjustSource !== calibrationSource) {
      controlFrame.adjustStartPosition.copy(calibrationSample.position);
      controlFrame.adjustStartQuaternion.copy(calibrationSample.rawQuaternion).normalize();
      controlFrame.hasAdjustReference = true;
      controlFrame.adjustSource = calibrationSource;
    } else {
      scratchVecA.copy(calibrationSample.position).sub(controlFrame.adjustStartPosition);
      controlFrame.position.add(scratchVecA);
      controlFrame.adjustStartPosition.copy(calibrationSample.position);

      const currentYaw = extractYawRadians(calibrationSample.rawQuaternion);
      const startYaw = extractYawRadians(controlFrame.adjustStartQuaternion);
      const deltaYaw = normalizeAngleRadians(currentYaw - startYaw);
      controlFrame.quaternion.premultiply(setYawQuaternion(scratchQuatC, deltaYaw)).normalize();
      controlFrame.adjustStartQuaternion.copy(calibrationSample.rawQuaternion).normalize();
    }
    controlFrame.active = true;
    controlFrame.fadeAlpha = 1;
    return;
  }

  if (controlFrame.active) {
    controlFrame.active = false;
    controlFrame.hasAdjustReference = false;
    controlFrame.adjustSource = null;
    controlFrame.lastReleaseMs = Date.now();
  }
}

function updateControlFrameVisual(nowMs: number) {
  if (controlFrame.active) {
    controlFrame.fadeAlpha = 1;
  } else if (controlFrame.lastReleaseMs > 0) {
    const elapsed = nowMs - controlFrame.lastReleaseMs;
    controlFrame.fadeAlpha = elapsed >= AXIS_FADE_MS ? 0 : 1 - elapsed / AXIS_FADE_MS;
  } else {
    controlFrame.fadeAlpha = 0;
  }

  originVisualRoot.visible = controlFrame.fadeAlpha > 0.01;
  originVisualRoot.position.copy(controlFrame.position);
  originVisualRoot.quaternion.copy(controlFrame.quaternion);
  setVisualOpacity(originVisualRoot, controlFrame.fadeAlpha);
}

function handlePanelInput(rightSample: ControllerFrameSample | null) {
  const nowMs = Date.now();
  panelInteraction.hoveredKeyButtonId = 0;
  panelInteraction.activeKeyButtonId = 0;
  panelInteraction.keyFlags = 0;

  if (!rightSample?.connected) {
    buttonHistory.right.primary = false;
    buttonHistory.right.secondary = false;
    buttonHistory.right.primaryStartedAtMs = 0;
    buttonHistory.right.secondaryStartedAtMs = 0;
    buttonHistory.right.primaryLongHoldActive = false;
    buttonHistory.right.secondaryLongHoldActive = false;
    return;
  }

  const triggerPressed = rightSample.triggerValue >= TRIGGER_PRESS_THRESHOLD;
  const squeezePressed = rightSample.squeezeValue >= 0.55;

  if (squeezePressed && rightSample.primaryPressed && !buttonHistory.right.primary) {
    buttonHistory.right.primaryStartedAtMs = nowMs;
    buttonHistory.right.primaryLongHoldActive = false;
  }
  if (squeezePressed && rightSample.secondaryPressed && !buttonHistory.right.secondary) {
    buttonHistory.right.secondaryStartedAtMs = nowMs;
    buttonHistory.right.secondaryLongHoldActive = false;
  }

  if (!squeezePressed) {
    buttonHistory.right.primaryStartedAtMs = 0;
    buttonHistory.right.secondaryStartedAtMs = 0;
    buttonHistory.right.primaryLongHoldActive = false;
    buttonHistory.right.secondaryLongHoldActive = false;
  } else {
    if (rightSample.primaryPressed && !buttonHistory.right.primaryLongHoldActive) {
      const heldMs = nowMs - buttonHistory.right.primaryStartedAtMs;
      if (buttonHistory.right.primaryStartedAtMs > 0 && heldMs >= PANEL_HOLD_MS) {
        buttonHistory.right.primaryLongHoldActive = true;
      }
    }
    if (rightSample.secondaryPressed && !buttonHistory.right.secondaryLongHoldActive) {
      const heldMs = nowMs - buttonHistory.right.secondaryStartedAtMs;
      if (buttonHistory.right.secondaryStartedAtMs > 0 && heldMs >= PANEL_HOLD_MS) {
        buttonHistory.right.secondaryLongHoldActive = true;
      }
    }
  }

  if (squeezePressed && buttonHistory.right.primaryLongHoldActive) {
    movePanelToController(panels.debug, rightSample, 0.44, 0.06, -0.18);
    panels.debug.visible = true;
  }
  if (squeezePressed && buttonHistory.right.secondaryLongHoldActive) {
    movePanelToController(panels.keys, rightSample, 0.44, -0.03, 0.18);
    panels.keys.visible = true;
  }

  if (buttonHistory.right.primary && !rightSample.primaryPressed) {
    const releasedAfterSqueeze = squeezePressed && buttonHistory.right.primaryStartedAtMs > 0;
    if (releasedAfterSqueeze && !buttonHistory.right.primaryLongHoldActive) {
      togglePanel('debug', rightSample);
    }
    buttonHistory.right.primaryStartedAtMs = 0;
    buttonHistory.right.primaryLongHoldActive = false;
  }
  if (buttonHistory.right.secondary && !rightSample.secondaryPressed) {
    const releasedAfterSqueeze = squeezePressed && buttonHistory.right.secondaryStartedAtMs > 0;
    if (releasedAfterSqueeze && !buttonHistory.right.secondaryLongHoldActive) {
      togglePanel('keys', rightSample);
    }
    buttonHistory.right.secondaryStartedAtMs = 0;
    buttonHistory.right.secondaryLongHoldActive = false;
  }

  if (panels.keys.visible) {
    const hit = raycastPanelButton(panels.keys, rightSample);
    if (hit) {
      panelInteraction.hoveredKeyButtonId = hit.id;
      if (triggerPressed) {
        panelInteraction.activeKeyButtonId = hit.id;
        panelInteraction.keyFlags |= hit.bitMask;
      }
    }
  }

  if (!squeezePressed) {
    if (rightSample.primaryPressed) {
      panelInteraction.keyFlags |= KEY6_BIT;
    }
    if (rightSample.secondaryPressed) {
      panelInteraction.keyFlags |= KEY7_BIT;
    }
  }

  if (triggerPressed && panelInteraction.activeKeyButtonId === 0) {
    panelInteraction.keyFlags |= DELTA_KEY_BIT;
  }

  buttonHistory.right.primary = rightSample.primaryPressed;
  buttonHistory.right.secondary = rightSample.secondaryPressed;
}

function togglePanel(key: keyof typeof panels, sample: ControllerFrameSample) {
  const panel = panels[key];
  panel.visible = !panel.visible;
  if (panel.visible) {
    movePanelToController(
      panel,
      sample,
      key === 'info' ? 0.52 : 0.42,
      key === 'debug' ? 0.06 : key === 'keys' ? -0.03 : 0.02,
      key === 'debug' ? -0.18 : key === 'keys' ? 0.18 : 0
    );
  }
}

function movePanelToController(panel: WorldPanel, sample: ControllerFrameSample, distance: number, upOffset: number, sideOffset: number) {
  scratchVecA.copy(txLocalAxisY).applyQuaternion(sample.quaternion).multiplyScalar(distance);
  scratchVecB.copy(txLocalAxisZ).applyQuaternion(sample.quaternion).multiplyScalar(upOffset);
  scratchVecC.copy(txLocalAxisX).applyQuaternion(sample.quaternion).multiplyScalar(sideOffset);

  panel.root.position.copy(sample.position).add(scratchVecA).add(scratchVecB).add(scratchVecC);
  getViewerWorldPosition(scratchHead);
  panel.root.lookAt(scratchHead);
}

function raycastPanelButton(panel: WorldPanel, sample: ControllerFrameSample): PanelButton | null {
  scratchVecA.copy(sample.position);
  scratchVecB.copy(txLocalAxisY).applyQuaternion(sample.quaternion).normalize();
  raycaster.set(scratchVecA, scratchVecB);
  const intersections = raycaster.intersectObject(panel.mesh, false);
  if (intersections.length === 0) {
    return null;
  }

  const uv = intersections[0].uv;
  if (!uv) {
    return null;
  }

  const px = uv.x * panel.width;
  const py = (1 - uv.y) * panel.height;
  return (
    panel.buttons.find((button) => pointInRect(px, py, button.rect.x, button.rect.y, button.rect.width, button.rect.height)) ?? null
  );
}

function drawPanels() {
  panels.debug.root.visible = panels.debug.visible;
  panels.keys.root.visible = panels.keys.visible;
  panels.info.root.visible = panels.info.visible;

  if (panels.debug.visible) {
    drawDebugPanel(panels.debug);
  }
  if (panels.keys.visible) {
    drawKeyPanel(panels.keys);
  }
  if (panels.info.visible) {
    drawInfoPanel(panels.info);
  }
}

function drawDebugPanel(panel: WorldPanel) {
  const ctx = panel.canvas.getContext('2d');
  if (!ctx) {
    return;
  }

  const right = controllerState.right;
  const left = controllerState.left;
  preparePanelCanvas(ctx, panel);
  drawPanelTitle(ctx, panel);
  drawPanelParagraph(ctx, 54, 126, '');
  drawPanelParagraph(ctx, 54, 126, '');

  drawPanelParagraph(ctx, 54, 126, '');

  const lines = [
    `右手连接: ${right.connected ? '是' : '否'}    左手连接: ${left.connected ? '是' : '否'}`,
    `手势功能: ${handGestureState.featureEnabled ? '启用' : '禁用'}  跟踪: ${boolLabel(handGestureState.handTracked)}  模式: ${boolLabel(handGestureState.active)}`,
    `校准模式: ${boolLabel(runtimeConfigView.calibrationModeEnabled)}  左手手势校准: ${runtimeConfigView.calibrationModeEnabled ? '可用' : '关闭'}`,
    `ABS 位置(mm): ${formatVec3(right.rawPositionMm)}`,
    `REL 位置(mm): ${formatVec3(right.relPositionMm)}`,
    `ABS 四元数(wxyz): ${formatQuat(right.rawQuaternionWxyz)}`,
    `REL 四元数(wxyz): ${formatQuat(right.relQuaternionWxyz)}`,
    `摇杆: (${right.joyX}, ${right.joyY})    Trigger=${right.triggerValue.toFixed(2)}    Squeeze=${right.squeezeValue.toFixed(2)}`,
    `A=${boolLabel(right.primaryPressed)}  B=${boolLabel(right.secondaryPressed)}  Stick=${boolLabel(right.thumbstickPressed)}`,
    `掌心向上=${boolLabel(handGestureState.palmUp)}  展平=${boolLabel(handGestureState.openHand)}  静止=${boolLabel(handGestureState.still)}  W(KEY6)=${boolLabel(handGestureState.touchMiddle)}  X(KEY7)=${boolLabel(handGestureState.touchRing)}`
  ];

  drawKeyValueLines(ctx, lines, 54, 192, 42);
  panel.texture.needsUpdate = true;
}

function drawKeyPanel(panel: WorldPanel) {
  const ctx = panel.canvas.getContext('2d');
  if (!ctx) {
    return;
  }

  preparePanelCanvas(ctx, panel);
  drawPanelTitle(ctx, panel);
  drawPanelParagraph(ctx, 54, 126, '');
  drawPanelParagraph(ctx, 54, 126, '');

  drawPanelParagraph(ctx, 54, 126, '');

  for (const button of panel.buttons) {
    const hovered = panelInteraction.hoveredKeyButtonId === button.id;
    const active = panelInteraction.activeKeyButtonId === button.id;
    drawVirtualKeyButton(ctx, button, hovered, active);
  }

  ctx.fillStyle = '#cbd5e1';
  ctx.font = '26px "Microsoft YaHei", "PingFang SC", sans-serif';
  ctx.fillText(`当前 keyFlags: 0x${panelInteraction.keyFlags.toString(16).padStart(2, '0').toUpperCase()}`, 84, 582);
  panel.texture.needsUpdate = true;
}

function drawInfoPanel(panel: WorldPanel) {
  const ctx = panel.canvas.getContext('2d');
  if (!ctx) {
    return;
  }

  const fade = controlFrame.fadeAlpha;
  preparePanelCanvas(ctx, panel);
  drawPanelTitle(ctx, panel);
  drawPanelParagraph(ctx, 54, 126, 'A+B 切换本面板；A/B + Squeeze 可移动其它面板；Trigger 映射为 KEY5。');

  drawPanelParagraph(ctx, 54, 126, '纯手势模式下可启用右手手势按键：掌心向上、五指完全展平并近乎静止 300ms 后激活；激活后按更宽松阈值保持。');

  const sections = [
    `传输: ${transport.statusLabel}    模式: ${currentSessionMode.value}`,
    `发送频率: ${sendHz} Hz    坐标轴淡出: ${(fade * 100).toFixed(0)}%`,
    `PC 倍率: 平移 x${runtimeConfigView.positionScale.toFixed(2)} / 旋转 x${runtimeConfigView.rotationScale.toFixed(2)}`,
    `校准模式: ${boolLabel(runtimeConfigView.calibrationModeEnabled)}  左手捏合可做无手柄坐标校准`,
    `坐标系原点(mm): ${formatVec3(vectorMetersToMmTuple(controlFrame.position))}`,
    `坐标系四元数(wxyz): ${formatQuat(threeQuatToWxyzTuple(controlFrame.quaternion))}`,
    `面板: 调试=${boolLabel(panels.debug.visible)}  按键=${boolLabel(panels.keys.visible)}  状态=${boolLabel(panels.info.visible)}`,
    `右手 keyFlags: 0x${panelInteraction.keyFlags.toString(16).padStart(2, '0').toUpperCase()}`,
    `手势功能: ${handGestureState.featureEnabled ? '启用' : '禁用'}  跟踪=${boolLabel(handGestureState.handTracked)}  模式=${boolLabel(handGestureState.active)}`
  ];

  drawKeyValueLines(ctx, sections, 54, 194, 44);
  panel.texture.needsUpdate = true;
}

function updateHandGestureVisual(sample: HandFrameSample | null) {
  handGestureVisual.root.visible = Boolean(sample && handGestureState.active);
  if (!sample || !handGestureState.active) {
    return;
  }

  handGestureVisual.root.position.set(0, 0, 0);
  handGestureVisual.header.position.copy(sample.wrist);
  handGestureVisual.header.position.y += 0.11;
  redrawSpriteTag(handGestureVisual.header, '右手手势按键', '#22c55e');
  for (const finger of HAND_GESTURE_FINGERS) {
    handGestureVisual.markers[finger].position.copy(sample.fingertips[finger]);
  }

  handGestureVisual.keyLabels.middle.position.copy(sample.fingertips.middle).add(new THREE.Vector3(0.02, 0.018, 0));
  handGestureVisual.keyLabels.ring.position.copy(sample.fingertips.ring).add(new THREE.Vector3(0.02, 0.018, 0));
  redrawCompactKeyTag(handGestureVisual.keyLabels.middle, 'KEY6', '#22c55e', handGestureState.touchMiddle);
  redrawCompactKeyTag(handGestureVisual.keyLabels.ring, 'KEY7', '#eab308', handGestureState.touchRing);

  setFingerMarkerFeedback(handGestureVisual.markers.thumb, handGestureState.touchMiddle || handGestureState.touchRing);
  setFingerMarkerFeedback(handGestureVisual.markers.middle, handGestureState.touchMiddle);
  setFingerMarkerFeedback(handGestureVisual.markers.ring, handGestureState.touchRing);
  setFingerMarkerFeedback(handGestureVisual.markers.index, false);
  setFingerMarkerFeedback(handGestureVisual.markers.pinky, false);
}

function updateControllerVisual(handedness: Handedness, sample: ControllerFrameSample) {
  const visual = controllerVisuals[handedness];
  visual.root.visible = true;
  visual.root.position.copy(sample.position);
  visual.root.quaternion.copy(sample.quaternion);
  visual.root.scale.setScalar(1);

  const triggerBlend = clampNumber(sample.triggerValue / TRIGGER_PRESS_THRESHOLD, 0, 1);
  visual.directionArrowMaterial.color
    .copy(visual.directionArrowIdleColor)
    .lerp(visual.directionArrowPressedColor, triggerBlend);
  visual.directionArrowMaterial.emissive
    .copy(visual.directionArrowIdleColor)
    .lerp(visual.directionArrowPressedColor, triggerBlend);
  visual.directionArrowMaterial.emissiveIntensity = 0.24 + triggerBlend * 0.86;

  const accent = handedness === 'left' ? '#38bdf8' : '#f97316';
  redrawSpriteTag(visual.header, handedness === 'left' ? '左手' : '右手', accent);
}

function updateDesktopPreview(timeMs: number) {
  const t = timeMs * 0.001;
  camera.position.x = Math.sin(t * 0.18) * 2.25;
  camera.position.z = Math.cos(t * 0.18) * 2.25;
  camera.lookAt(0, 1.18, 0);
  desktopRig.visible = true;
  panels.debug.root.visible = false;
  panels.keys.root.visible = false;
  panels.info.root.visible = false;
  originVisualRoot.visible = false;
  handGestureVisual.root.visible = false;
  for (const handedness of ['left', 'right'] as const) {
    controllerVisuals[handedness].root.visible = false;
  }
}

function buildWireController(handedness: Handedness): WireController {
  const state = controllerState[handedness];
  const keyFlags = handedness === 'right' ? panelInteraction.keyFlags : 0;
  return {
    connected: state.connected,
    // Wire pose is always expressed in the calibrated control frame so the
    // backend/lower machine uses the redefined origin and axis directions.
    absPositionMm: [...state.txPositionMm],
    relPositionMm: [...state.txPositionMm],
    absQuaternionWxyz: [...state.txQuaternionWxyz],
    relQuaternionWxyz: [...state.txQuaternionWxyz],
    joy: {
      x: state.joyX,
      y: state.joyY
    },
    buttons: {
      trigger: round3(state.triggerValue),
      squeeze: round3(state.squeezeValue),
      thumbstick: state.thumbstickPressed,
      a: state.primaryPressed,
      b: state.secondaryPressed
    },
    keyFlags,
    panelState: {
      debug: panels.debug.visible,
      keys: panels.keys.visible,
      info: panels.info.visible
    }
  };
}

function updateUiSummary() {
  socketStatusEl.textContent = transport.statusLabel;

  const right = controllerState.right;
  lastFrameEl.textContent = [
    `WS: ${wsUrl}`,
    `运行模式: ${currentSessionMode.value}`,
    `右手连接: ${right.connected ? '是' : '否'}`,
    `手势功能: ${handGestureState.featureEnabled ? '启用' : '禁用'} / 跟踪=${boolLabel(handGestureState.handTracked)} / 模式=${boolLabel(handGestureState.active)}`,
    `校准模式: ${boolLabel(runtimeConfigView.calibrationModeEnabled)}`,
    `手势状态: 掌心向上=${boolLabel(handGestureState.palmUp)} / 展平=${boolLabel(handGestureState.openHand)} / 静止=${boolLabel(handGestureState.still)} / W=${boolLabel(handGestureState.touchMiddle)} / X=${boolLabel(handGestureState.touchRing)}`,
    `坐标系原点(mm): ${formatVec3(vectorMetersToMmTuple(controlFrame.position))}`,
    `坐标系淡出: ${(controlFrame.fadeAlpha * 100).toFixed(0)}%`,
    `摇杆: (${right.joyX}, ${right.joyY})`,
    `keyFlags: 0x${panelInteraction.keyFlags.toString(16).padStart(2, '0').toUpperCase()}`,
    '',
    '控制说明：',
    '- 按住摇杆并移动/转动右手柄：增量调整坐标系，松开后 5 秒淡出',
    '- A：切换调试面板，A+Squeeze：移动调试面板',
    '- B：切换按键面板，B+Squeeze：移动按键面板',
    '- A+B：切换状态面板，A+B+Squeeze：移动状态面板',
    '- Trigger：发送 KEY5 / deltaKey',
    `- 右手纯手势模式${ENABLE_HAND_GESTURE_KEYS ? '已启用' : '已禁用'}：掌心向上、五指完全展平并近乎静止 300ms 后激活；激活后拇指碰中指=KEY6，碰无名指=KEY7`,
    '- 开启“校准模式”后，左手拇指-食指捏合作为无手柄坐标校准使能；左右两种校准都只调整位置 + yaw'
  ].join('\n');
}

function applySessionVisuals(mode: SessionMode) {
  const isImmersive = mode !== 'desktop';
  const isAr = mode === 'immersive-ar';

  currentSessionMode.value = mode;
  scene.background = isAr ? null : desktopBackground;
  scene.fog = isAr ? null : desktopFog;
  renderer.setClearAlpha(isAr ? 0 : 1);
  desktopRig.visible = !isImmersive;
  document.body.classList.toggle('xr-active', isImmersive);

  modeStatusEl.textContent = mode === 'desktop' ? '桌面预览' : isAr ? 'XR 透视' : '沉浸式 VR';
  xrButtonEl.textContent = isImmersive ? '退出 XR' : xrSupport.ar ? '进入 XR' : xrSupport.vr ? '进入 VR' : 'XR 不可用';
  xrButtonEl.disabled = !isImmersive && !(xrSupport.ar || xrSupport.vr);
}

function createControllerVisual(handedness: Handedness): ControllerVisual {
  const root = new THREE.Group();
  root.visible = false;

  const accent = handedness === 'left' ? 0x38bdf8 : 0xf97316;

  const body = new THREE.Group();
  body.position.set(0, -0.01, 0);
  // Rotate the controller body so its long axis matches local +Y forward.
  body.rotation.x = -Math.PI / 2;
  root.add(body);

  const shell = new THREE.Mesh(
    new THREE.CylinderGeometry(0.017, 0.022, 0.12, 18),
    new THREE.MeshStandardMaterial({
      color: accent,
      roughness: 0.36,
      metalness: 0.08
    })
  );
  body.add(shell);

  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.045, 0.006, 10, 32),
    new THREE.MeshStandardMaterial({
      color: 0xe2e8f0,
      emissive: accent,
      emissiveIntensity: 0.22,
      roughness: 0.5
    })
  );
  ring.rotation.x = Math.PI / 2;
  ring.position.set(0, 0.016, 0);
  body.add(ring);

  const directionArrowIdleColor = new THREE.Color(handedness === 'left' ? 0x7dd3fc : 0xfdba74);
  const directionArrowPressedColor = new THREE.Color(0xfacc15);
  const directionArrowMaterial = new THREE.MeshStandardMaterial({
    color: directionArrowIdleColor.clone(),
    emissive: directionArrowIdleColor.clone(),
    emissiveIntensity: 0.24,
    roughness: 0.28,
    metalness: 0.06
  });
  const directionArrow = new THREE.Mesh(
    new THREE.ConeGeometry(0.018, 0.05, 24),
    directionArrowMaterial
  );
  directionArrow.position.set(0, 0.085, 0);
  body.add(directionArrow);

  // Display the handle/local attitude frame directly: X right, Y forward, Z up.
  root.add(makeArrow(txLocalAxisX, 0xff4d4f));
  root.add(makeArrow(txLocalAxisY, 0x22c55e));
  root.add(makeArrow(txLocalAxisZ, 0x3b82f6));

  const ray = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), txLocalAxisY.clone().multiplyScalar(0.28)]),
    new THREE.LineBasicMaterial({
      color: handedness === 'left' ? 0x67e8f9 : 0xfdba74,
      transparent: true,
      opacity: 0.78
    })
  );
  root.add(ray);

  const header = createFloatingTag(handedness === 'left' ? '左手' : '右手', handedness === 'left' ? '#38bdf8' : '#f97316');
  header.position.set(0, 0.19, 0.02);
  root.add(header);

  primeMaterialOpacity(root);
  return {
    root,
    header,
    directionArrowMaterial,
    directionArrowIdleColor,
    directionArrowPressedColor
  };
}

function createHandGestureVisual(): HandGestureVisual {
  const root = new THREE.Group();
  root.visible = false;

  const header = createFloatingTag('右手手势按键', '#22c55e');
  root.add(header);

  const markers = {
    thumb: createFingerMarker(0xf97316),
    index: createFingerMarker(0x38bdf8),
    middle: createFingerMarker(0x22c55e),
    ring: createFingerMarker(0xeab308),
    pinky: createFingerMarker(0xa855f7)
  } satisfies Record<HandGestureFingerName, THREE.Mesh<THREE.SphereGeometry, THREE.MeshStandardMaterial>>;

  for (const finger of HAND_GESTURE_FINGERS) {
    root.add(markers[finger]);
  }

  const keyLabels = {
    middle: createCompactKeyTag('KEY6', '#22c55e'),
    ring: createCompactKeyTag('KEY7', '#eab308')
  };
  root.add(keyLabels.middle, keyLabels.ring);

  primeMaterialOpacity(root);
  return { root, header, markers, keyLabels };
}

function createFingerMarker(color: number) {
  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(0.011, 18, 18),
    new THREE.MeshStandardMaterial({
      color,
      emissive: color,
      emissiveIntensity: 0.42,
      roughness: 0.32,
      metalness: 0.08
    })
  );
  marker.scale.setScalar(HAND_GESTURE_IDLE_MARKER_SCALE);
  return marker;
}

function buildControlFrameVisual() {
  const root = new THREE.Group();

  // GridHelper is native-XZ/y=0, which is exactly the transmitted world
  // frame's horizontal XY plane because outgoing +Z maps to native +Y.
  const grid = new THREE.GridHelper(1.3, 14, 0x7dd3fc, 0x1f3145);
  grid.position.y = -0.001;
  root.add(grid);

  root.add(makeArrow(txWorldAxisX, 0xff4d4f));
  root.add(makeArrow(txWorldAxisY, 0x22c55e));
  root.add(makeArrow(txWorldAxisZ, 0x3b82f6));

  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(0.022, 18, 18),
    new THREE.MeshStandardMaterial({
      color: 0xf8fafc,
      emissive: 0x38bdf8,
      emissiveIntensity: 0.9,
      roughness: 0.24,
      metalness: 0.12
    })
  );
  root.add(marker);

  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.22, 0.007, 8, 40),
    new THREE.MeshBasicMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.82
    })
  );
  ring.rotation.x = Math.PI / 2;
  root.add(ring);

  const label = createFloatingTag('XR 坐标系', '#38bdf8');
  label.position.set(0, 0.18, 0.02);
  root.add(label);

  return root;
}

function createWorldPanel(
  key: WorldPanel['key'],
  title: string,
  accent: string,
  width: number,
  height: number,
  buttons: PanelButton[] = []
): WorldPanel {
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.minFilter = THREE.LinearFilter;
  texture.generateMipmaps = false;

  const material = new THREE.MeshBasicMaterial({
    map: texture,
    transparent: true,
    side: THREE.DoubleSide,
    depthWrite: false
  });

  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width / 2400, height / 2400), material);
  const root = new THREE.Group();
  root.visible = false;
  root.add(mesh);

  return {
    key,
    root,
    mesh,
    canvas,
    texture,
    width,
    height,
    accent,
    title,
    visible: false,
    buttons
  };
}

function preparePanelCanvas(ctx: CanvasRenderingContext2D, panel: WorldPanel) {
  ctx.clearRect(0, 0, panel.width, panel.height);
  ctx.fillStyle = 'rgba(5, 11, 24, 0.92)';
  roundRect(ctx, 16, 16, panel.width - 32, panel.height - 32, 36);
  ctx.fill();
  ctx.strokeStyle = panel.accent;
  ctx.lineWidth = 5;
  roundRect(ctx, 16, 16, panel.width - 32, panel.height - 32, 36);
  ctx.stroke();
}

function drawPanelTitle(ctx: CanvasRenderingContext2D, panel: WorldPanel) {
  ctx.fillStyle = panel.accent;
  ctx.font = '600 30px "Microsoft YaHei", "PingFang SC", sans-serif';
  ctx.fillText('CCtrl XR-UART', 54, 62);

  ctx.fillStyle = '#f8fafc';
  ctx.font = '700 44px "Microsoft YaHei", "PingFang SC", sans-serif';
  ctx.fillText(panel.title, 54, 104);
}

function drawPanelParagraph(ctx: CanvasRenderingContext2D, x: number, y: number, text: string) {
  ctx.fillStyle = '#cbd5e1';
  ctx.font = '26px "Microsoft YaHei", "PingFang SC", sans-serif';
  ctx.fillText(text, x, y);
}

function drawKeyValueLines(ctx: CanvasRenderingContext2D, lines: string[], x: number, startY: number, step: number) {
  ctx.fillStyle = '#e2e8f0';
  ctx.font = '28px "Consolas", "Microsoft YaHei UI", monospace';
  lines.forEach((line, index) => {
    ctx.fillText(line, x, startY + index * step);
  });
}

function drawVirtualKeyButton(ctx: CanvasRenderingContext2D, button: PanelButton, hovered: boolean, active: boolean) {
  ctx.fillStyle = active ? 'rgba(34, 197, 94, 0.92)' : hovered ? 'rgba(34, 197, 94, 0.36)' : 'rgba(15, 23, 42, 0.82)';
  roundRect(ctx, button.rect.x, button.rect.y, button.rect.width, button.rect.height, 28);
  ctx.fill();

  ctx.strokeStyle = active ? '#bbf7d0' : hovered ? '#86efac' : '#4ade80';
  ctx.lineWidth = active ? 6 : 4;
  roundRect(ctx, button.rect.x, button.rect.y, button.rect.width, button.rect.height, 28);
  ctx.stroke();

  const centerX = button.rect.x + button.rect.width / 2;
  const centerY = button.rect.y + button.rect.height / 2;
  ctx.fillStyle = active ? '#052e16' : '#f8fafc';
  ctx.font = '700 40px "Microsoft YaHei", "PingFang SC", sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'alphabetic';
  ctx.fillText(button.label, centerX, centerY - 4);
  ctx.fillStyle = active ? '#166534' : '#cbd5e1';
  ctx.font = '600 20px "Microsoft YaHei", "PingFang SC", sans-serif';
  ctx.fillText(button.subLabel, centerX, centerY + 28);
  ctx.textAlign = 'left';
  ctx.textBaseline = 'alphabetic';
}

function createFloatingTag(label: string, accent: string) {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 128;
  const ctx = canvas.getContext('2d');
  if (!ctx) {
    throw new Error('Canvas 2D unavailable');
  }

  ctx.fillStyle = 'rgba(4, 10, 22, 0.82)';
  roundRect(ctx, 12, 12, 488, 104, 26);
  ctx.fill();
  ctx.strokeStyle = accent;
  ctx.lineWidth = 4;
  roundRect(ctx, 12, 12, 488, 104, 26);
  ctx.stroke();

  ctx.fillStyle = '#f8fafc';
  ctx.font = '700 38px "Microsoft YaHei", "PingFang SC", sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, 256, 64);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthWrite: false
    })
  );
  sprite.scale.set(0.24, 0.06, 1);
  return sprite;
}

function createCompactKeyTag(label: string, accent: string) {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 92;
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthWrite: false
    })
  );
  sprite.scale.set(0.115, 0.042, 1);
  redrawCompactKeyTag(sprite, label, accent, false);
  return sprite;
}

function redrawSpriteTag(sprite: THREE.Sprite, label: string, accent: string) {
  const material = sprite.material as THREE.SpriteMaterial;
  const texture = material.map;
  if (!(texture instanceof THREE.CanvasTexture)) {
    return;
  }

  const canvas = texture.image as HTMLCanvasElement;
  const ctx = canvas.getContext('2d');
  if (!ctx) {
    return;
  }

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = 'rgba(4, 10, 22, 0.82)';
  roundRect(ctx, 12, 12, 488, 104, 26);
  ctx.fill();
  ctx.strokeStyle = accent;
  ctx.lineWidth = 4;
  roundRect(ctx, 12, 12, 488, 104, 26);
  ctx.stroke();

  ctx.fillStyle = '#f8fafc';
  ctx.font = '700 38px "Microsoft YaHei", "PingFang SC", sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, 256, 64);
  texture.needsUpdate = true;
}

function redrawCompactKeyTag(sprite: THREE.Sprite, label: string, accent: string, active: boolean) {
  const material = sprite.material as THREE.SpriteMaterial;
  const texture = material.map;
  if (!(texture instanceof THREE.CanvasTexture)) {
    return;
  }

  const canvas = texture.image as HTMLCanvasElement;
  const ctx = canvas.getContext('2d');
  if (!ctx) {
    return;
  }

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = active ? accent : 'rgba(4, 10, 22, 0.88)';
  roundRect(ctx, 10, 12, 236, 68, 22);
  ctx.fill();
  ctx.strokeStyle = active ? '#dcfce7' : accent;
  ctx.lineWidth = active ? 5 : 3;
  roundRect(ctx, 10, 12, 236, 68, 22);
  ctx.stroke();

  ctx.fillStyle = active ? '#052e16' : '#f8fafc';
  ctx.font = '700 28px "Microsoft YaHei", "PingFang SC", sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, 128, 48);
  texture.needsUpdate = true;
}

function makeArrow(direction: THREE.Vector3, color: number) {
  return new THREE.ArrowHelper(direction.clone().normalize(), new THREE.Vector3(), 0.14, color, 0.03, 0.018);
}

function createEmptyState(): ControllerState {
  return {
    connected: false,
    lastPoseSeenMs: 0,
    rawPositionMm: [0, 0, 0],
    relPositionMm: [0, 0, 0],
    txPositionMm: [0, 0, 0],
    rawQuaternionWxyz: [1, 0, 0, 0],
    relQuaternionWxyz: [1, 0, 0, 0],
    txQuaternionWxyz: [1, 0, 0, 0],
    axes: [],
    buttonsRaw: [],
    triggerValue: 0,
    squeezeValue: 0,
    thumbstickPressed: false,
    primaryPressed: false,
    secondaryPressed: false,
    joyX: 50,
    joyY: 50
  };
}

function setControllerDisconnected(handedness: Handedness) {
  controllerState[handedness] = createEmptyState();
  controllerVisuals[handedness].root.visible = false;
}

function resetControllers() {
  setControllerDisconnected('left');
  setControllerDisconnected('right');
  handGestureVisual.root.visible = false;
  panelInteraction.activeKeyButtonId = 0;
  panelInteraction.hoveredKeyButtonId = 0;
  panelInteraction.keyFlags = 0;
  buttonHistory.right.primary = false;
  buttonHistory.right.secondary = false;
  buttonHistory.right.primaryStartedAtMs = 0;
  buttonHistory.right.secondaryStartedAtMs = 0;
  buttonHistory.right.primaryLongHoldActive = false;
  buttonHistory.right.secondaryLongHoldActive = false;
  handGestureState.handTracked = false;
  handGestureState.palmUp = false;
  handGestureState.openHand = false;
  handGestureState.still = false;
  handGestureState.eligible = false;
  handGestureState.active = false;
  handGestureState.eligibleSinceMs = 0;
  handGestureState.ineligibleSinceMs = 0;
  handGestureState.touchMiddle = false;
  handGestureState.touchRing = false;
  handGestureState.keyFlags = 0;
  handGestureState.hasPreviousPose = false;
}

function resetPanels() {
  panels.debug.visible = false;
  panels.keys.visible = false;
  panels.info.visible = false;
}

function resetControlFrame() {
  controlFrame.initialized = false;
  controlFrame.position.set(0, 0, 0);
  controlFrame.quaternion.identity();
  controlFrame.adjustStartPosition.set(0, 0, 0);
  controlFrame.adjustStartQuaternion.identity();
  controlFrame.hasAdjustReference = false;
  controlFrame.adjustSource = null;
  controlFrame.fadeAlpha = 0;
  controlFrame.lastReleaseMs = 0;
  controlFrame.active = false;
}

function setVisualOpacity(root: THREE.Object3D, factor: number) {
  root.traverse((object) => {
    if (!('material' in object)) {
      return;
    }

    const material = object.material;
    const materials = Array.isArray(material) ? material : [material];
    for (const entry of materials) {
      if (!entry) {
        continue;
      }
      const typed = entry as THREE.Material & { opacity: number; userData: { baseOpacity?: number } };
      const baseOpacity = typeof typed.userData.baseOpacity === 'number' ? typed.userData.baseOpacity : typed.opacity;
      typed.opacity = baseOpacity * factor;
      typed.transparent = true;
      typed.needsUpdate = true;
    }
  });
}

function primeMaterialOpacity(root: THREE.Object3D) {
  root.traverse((object) => {
    if (!('material' in object)) {
      return;
    }
    const material = object.material;
    const materials = Array.isArray(material) ? material : [material];
    for (const entry of materials) {
      if (!entry) {
        continue;
      }
      const typed = entry as THREE.Material & { opacity: number; userData: { baseOpacity?: number } };
      typed.userData.baseOpacity = typed.opacity;
      typed.transparent = true;
    }
  });
}

function getInputSourceHand(inputSource: XRInputSource) {
  if (!('hand' in inputSource)) {
    return null;
  }
  return inputSource.hand ?? null;
}

function readHandJointPosition(frame: XRFrame, hand: XRHand, jointName: HandJointName, referenceSpace: XRReferenceSpace) {
  const jointSpace = hand.get(jointName as XRHandJoint);
  if (!jointSpace) {
    return null;
  }

  const getJointPose = frame.getJointPose;
  if (!getJointPose) {
    return null;
  }

  const pose = getJointPose.call(frame, jointSpace, referenceSpace);
  if (!pose) {
    return null;
  }

  return new THREE.Vector3(pose.transform.position.x, pose.transform.position.y, pose.transform.position.z);
}

function isFingerExtended(
  wrist: THREE.Vector3,
  tip: THREE.Vector3,
  proximal: THREE.Vector3,
  margin = HAND_GESTURE_FINGER_EXTEND_MARGIN_M
) {
  return tip.distanceTo(wrist) - proximal.distanceTo(wrist) >= margin;
}

function isHandPoseStill(sample: HandFrameSample) {
  if (!handGestureState.hasPreviousPose) {
    return false;
  }

  let maxDelta = sample.wrist.distanceTo(handGestureState.previousWrist);
  for (const finger of HAND_GESTURE_FINGERS) {
    maxDelta = Math.max(maxDelta, sample.fingertips[finger].distanceTo(handGestureState.previousFingertips[finger]));
  }
  return maxDelta <= HAND_GESTURE_STILL_DELTA_M;
}

function rememberHandPose(sample: HandFrameSample) {
  handGestureState.previousWrist.copy(sample.wrist);
  for (const finger of HAND_GESTURE_FINGERS) {
    handGestureState.previousFingertips[finger].copy(sample.fingertips[finger]);
  }
  handGestureState.hasPreviousPose = true;
}

function extractYawRadians(quaternion: THREE.Quaternion) {
  scratchVecA.copy(txLocalAxisY).applyQuaternion(quaternion);
  return Math.atan2(-scratchVecA.x, -scratchVecA.z);
}

function setYawQuaternion(target: THREE.Quaternion, yawRadians: number) {
  return target.setFromAxisAngle(worldUp, yawRadians).normalize();
}

function normalizeAngleRadians(value: number) {
  let result = value;
  while (result > Math.PI) {
    result -= Math.PI * 2;
  }
  while (result < -Math.PI) {
    result += Math.PI * 2;
  }
  return result;
}

function setFingerMarkerFeedback(
  marker: THREE.Mesh<THREE.SphereGeometry, THREE.MeshStandardMaterial>,
  active: boolean
) {
  marker.scale.setScalar(active ? HAND_GESTURE_ACTIVE_MARKER_SCALE : HAND_GESTURE_IDLE_MARKER_SCALE);
  marker.material.emissiveIntensity = active ? 1.15 : 0.42;
}

function mustElement<T extends HTMLElement>(id: string) {
  const element = document.getElementById(id);
  if (!element) {
    throw new Error(`Element not found: ${id}`);
  }
  return element as T;
}

function resolveWsUrl(explicitUrl?: string) {
  if (explicitUrl && explicitUrl.trim().length > 0) {
    return explicitUrl;
  }

  const { protocol, host } = window.location;
  if (protocol === 'https:') {
    return `wss://${host}/ws`;
  }
  return 'ws://localhost:8787/ws';
}

function resolveStickAxis(axes: number[], axisIndex: 0 | 1) {
  if (axes.length === 0) {
    return 0;
  }
  const baseIndex = axes.length >= 2 ? axes.length - 2 : 0;
  return Number(axes[baseIndex + axisIndex] ?? 0);
}

function mapAxisToPercent(value: number) {
  return clampInt(Math.round(50 + clampNumber(value, -1, 1) * 50), 0, 100);
}

function vectorMetersToMmTuple(vector: THREE.Vector3): Vec3Tuple {
  // WebXR/OpenXR grip/world space uses +X right, +Y up, -Z forward.
  // CCtrl's XR-side world/position frame is defined as a left-handed Z-up frame:
  //   X' =  X
  //   Y' =  Z
  //   Z' =  Y
  return [
    round3(vector.x * 1000),
    round3(vector.z * 1000),
    round3(vector.y * 1000)
  ];
}

function threeQuatToWxyzTuple(quaternion: THREE.Quaternion): QuatWxyzTuple {
  // Handle/local attitude keeps the agreed IMU-like axis definition:
  //   +X = native +X
  //   +Y = native -Z
  //   +Z = native +Y
  return [
    round3(quaternion.w),
    round3(quaternion.x),
    round3(-quaternion.z),
    round3(quaternion.y)
  ];
}

function formatVec3(vector: Vec3Tuple) {
  return vector.map((value) => value.toFixed(1)).join('  ');
}

function formatQuat(quaternion: QuatWxyzTuple) {
  return quaternion.map((value) => value.toFixed(3)).join('  ');
}

function round3(value: number) {
  return Math.round(value * 1000) / 1000;
}

function clampNumber(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value));
}

function clampInt(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, Math.round(value)));
}

function pointInRect(x: number, y: number, left: number, top: number, width: number, height: number) {
  return x >= left && x <= left + width && y >= top && y <= top + height;
}

function boolLabel(value: boolean) {
  return value ? '开' : '关';
}

function getViewerWorldPosition(target: THREE.Vector3) {
  const xrCamera = renderer.xr.getCamera();
  xrCamera.getWorldPosition(target);
  return target;
}

function roundRect(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  width: number,
  height: number,
  radius: number
) {
  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.arcTo(x + width, y, x + width, y + height, radius);
  ctx.arcTo(x + width, y + height, x, y + height, radius);
  ctx.arcTo(x, y + height, x, y, radius);
  ctx.arcTo(x, y, x + width, y, radius);
  ctx.closePath();
}

async function refreshRuntimeConfig() {
  try {
    const response = await fetch('/runtime-config.json', { cache: 'no-store' });
    if (response.ok) {
      const data = (await response.json()) as Partial<{
        positionScale: number;
        rotationScale: number;
        calibrationModeEnabled: boolean;
      }>;
      runtimeConfigView.positionScale = Number(data.positionScale ?? 1);
      runtimeConfigView.rotationScale = Number(data.rotationScale ?? 1);
      runtimeConfigView.calibrationModeEnabled = Boolean(data.calibrationModeEnabled ?? false);
      runtimeConfigView.lastFetchedAtMs = Date.now();
    }
  } catch {
    // Ignore when using local Vite dev without the relay backend.
  } finally {
    window.setTimeout(() => {
      void refreshRuntimeConfig();
    }, 2000);
  }
}
