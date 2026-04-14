import {
  AmbientLight,
  Box3,
  Color,
  DirectionalLight,
  LoadingManager,
  Mesh,
  PerspectiveCamera,
  PlaneGeometry,
  Scene,
  ShadowMaterial,
  Vector3,
  WebGLRenderer,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from '/third_party/urdf-loaders/javascript/src/URDFLoader.js';

// DOM refs
const viewer = document.getElementById('viewer');
const statusEl = document.getElementById('status');
const robotNameEl = document.getElementById('robot-name');
const robotChainEl = document.getElementById('robot-chain');
const armBadgeEl = document.getElementById('arm-badge');
const jointsBodyEl = document.getElementById('joints-body');
const ikDotEl = document.getElementById('ik-dot');
const ikSummaryEl = document.getElementById('ik-summary');
const homeBtn = document.getElementById('btn-home');
const armBtn = document.getElementById('btn-arm');

const poseEls = {
  x: document.getElementById('pose-x'),
  y: document.getElementById('pose-y'),
  z: document.getElementById('pose-z'),
  roll: document.getElementById('pose-roll'),
  pitch: document.getElementById('pose-pitch'),
  yaw: document.getElementById('pose-yaw'),
};

// Keypad highlight map
const keyEls = new Map();
document.querySelectorAll('.key').forEach(el => {
  keyEls.set(el.dataset.code, el);
});

const activeKeys = new Set();
const acceptedKeys = new Set([
  'KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE',
  'KeyJ', 'KeyU', 'KeyK', 'KeyI', 'KeyL', 'KeyO',
]);
const switchArmKey = 'KeyC';
const resetKey = 'KeyR';

let meta = null;
let robot = null;
let robotRoot = null;
let renderer = null;
let scene = null;
let camera = null;
let controls = null;
let busy = false;

// ── Formatting ────────────────────────────────────────────────────────────────

const RAD2DEG = 180 / Math.PI;

function fmt(value, digits = 4) {
  return Number(value).toFixed(digits);
}

// ── State application ─────────────────────────────────────────────────────────

function applyPose(pose) {
  if (!pose || pose.length !== 6) return;
  poseEls.x.textContent = fmt(pose[0]);
  poseEls.y.textContent = fmt(pose[1]);
  poseEls.z.textContent = fmt(pose[2]);
  poseEls.roll.textContent  = fmt(pose[3] * RAD2DEG, 2) + '°';
  poseEls.pitch.textContent = fmt(pose[4] * RAD2DEG, 2) + '°';
  poseEls.yaw.textContent   = fmt(pose[5] * RAD2DEG, 2) + '°';
}

function applyJoints(jointNames, q) {
  if (!jointNames || !q || !meta) return;

  // Rebuild table rows when joint count changes (e.g. after arm switch).
  if (jointsBodyEl.childElementCount !== jointNames.length) {
    jointsBodyEl.innerHTML = '';
    for (let i = 0; i < jointNames.length; i += 1) {
      const row = document.createElement('tr');
      row.innerHTML = `
        <td>${jointNames[i]}</td>
        <td class="num" data-role="rad">-</td>
        <td class="num" data-role="deg">-</td>
        <td><div class="joint-bar"><span></span></div></td>
      `;
      jointsBodyEl.appendChild(row);
    }
  }

  const rows = jointsBodyEl.children;
  for (let i = 0; i < jointNames.length; i += 1) {
    const row = rows[i];
    if (!row) continue;
    const limits = meta.joint_limits?.[jointNames[i]] || [-Math.PI, Math.PI];
    const [lower, upper] = limits;
    const span = Math.max(upper - lower, 1e-6);
    const frac = Math.min(1, Math.max(0, (q[i] - lower) / span));

    row.querySelector('[data-role="rad"]').textContent = fmt(q[i], 4);
    row.querySelector('[data-role="deg"]').textContent = fmt(q[i] * RAD2DEG, 2) + '°';
    row.querySelector('.joint-bar > span').style.width = `${(frac * 100).toFixed(1)}%`;
  }
}

function applyIk(ik, success) {
  if (ik) {
    ikDotEl.classList.remove('good', 'bad');
    ikDotEl.classList.add(success === false ? 'bad' : 'good');
    ikSummaryEl.textContent =
      `iter=${ik.iterations} pos=${fmt(ik.position_error)} rot=${fmt(ik.orientation_error)}`;
  } else if (success === false) {
    ikDotEl.classList.remove('good');
    ikDotEl.classList.add('bad');
    ikSummaryEl.textContent = 'failed';
  }
}

function applyState(state) {
  if (!state) return;

  if (state.ee_pose) applyPose(state.ee_pose);
  if (state.joint_names && state.q) applyJoints(state.joint_names, state.q);
  if (state.ik || state.success === false) applyIk(state.ik, state.success);

  if (robot) {
    if (state.all_joints) {
      robot.setJointValues(state.all_joints);
    } else if (state.joint_names && state.q) {
      const joints = {};
      for (let i = 0; i < state.joint_names.length; i += 1) {
        joints[state.joint_names[i]] = state.q[i];
      }
      robot.setJointValues(joints);
    }
    robot.updateMatrixWorld(true);
  }
}

function applyMeta(nextMeta) {
  robotNameEl.textContent = nextMeta.robot_name;
  robotChainEl.textContent = `${nextMeta.base_link} → ${nextMeta.ee_link}`;
  armBadgeEl.textContent = nextMeta.active_arm;
  // Clear joint rows so they rebuild on next applyState call.
  jointsBodyEl.innerHTML = '';
}

// ── Network ───────────────────────────────────────────────────────────────────

async function fetchJson(url, options = {}) {
  const response = await fetch(url, options);
  if (!response.ok) throw new Error(`${url} → ${response.status}`);
  return response.json();
}

async function pollState() {
  try {
    const state = await fetchJson('/api/state');
    applyState(state);
  } catch (err) {
    statusEl.textContent = `poll failed: ${err.message}`;
  }
}

async function sendKeyboard() {
  if (busy || activeKeys.size === 0) return;
  busy = true;
  try {
    const payload = { keys: Array.from(activeKeys) };
    const state = await fetchJson('/api/keyboard', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    statusEl.textContent = state.success ? 'running' : 'ik failed';
    applyState(state);
  } catch (err) {
    statusEl.textContent = `keyboard failed: ${err.message}`;
  } finally {
    busy = false;
  }
}

async function doHome() {
  try {
    const state = await fetchJson('/api/home', { method: 'POST' });
    statusEl.textContent = 'home';
    applyState(state);
  } catch (err) {
    statusEl.textContent = `home failed: ${err.message}`;
  }
}

async function doSwitchArm() {
  if (!meta) return;
  const arms = meta.available_arms || [];
  if (arms.length < 2) return;
  const next = arms.find(a => a !== meta.active_arm) || arms[0];
  try {
    const prevUrdfUrl = meta.urdf_url;
    const response = await fetchJson('/api/switch_arm', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ arm: next }),
    });
    meta = response.meta;
    applyMeta(meta);
    if (meta.urdf_url !== prevUrdfUrl) await reloadRobot();
    applyState(response.state);
    statusEl.textContent = `arm: ${next}`;
  } catch (err) {
    statusEl.textContent = `switch failed: ${err.message}`;
  }
}

// ── Scene ─────────────────────────────────────────────────────────────────────

function initScene() {
  scene = new Scene();
  scene.background = new Color(0x0f172a);

  camera = new PerspectiveCamera(45, viewer.clientWidth / viewer.clientHeight, 0.01, 100);
  camera.position.set(2.2, 1.8, 2.2);

  renderer = new WebGLRenderer({ antialias: true });
  renderer.shadowMap.enabled = true;
  renderer.setSize(viewer.clientWidth, viewer.clientHeight);
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  viewer.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.6, 0);
  controls.update();

  const sun = new DirectionalLight(0xffffff, 1.2);
  sun.position.set(3, 5, 2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(1024, 1024);
  scene.add(sun);
  scene.add(new AmbientLight(0xffffff, 0.35));

  const ground = new Mesh(
    new PlaneGeometry(12, 12),
    new ShadowMaterial({ opacity: 0.25 }),
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  window.addEventListener('resize', () => {
    renderer.setSize(viewer.clientWidth, viewer.clientHeight);
    camera.aspect = viewer.clientWidth / viewer.clientHeight;
    camera.updateProjectionMatrix();
  });
}

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}

async function reloadRobot() {
  if (robotRoot) {
    scene.remove(robotRoot);
    robotRoot = null;
    robot = null;
  }

  const manager = new LoadingManager();
  const loader = new URDFLoader(manager);

  // Wait for URDF parse + all mesh assets to finish loading.
  const loaded = await new Promise((resolve, reject) => {
    manager.onError = reject;
    loader.load(meta.urdf_url, robot => {
      new Promise(res => {
        if (manager.itemsLoaded >= manager.itemsTotal) { res(); }
        else { manager.onLoad = res; }
      }).then(() => resolve(robot));
    }, undefined, reject);
  });

  // ROS is Z-up; Three.js is Y-up. Rotate -90° around X to stand the robot upright.
  loaded.rotation.x = -Math.PI / 2;

  loaded.traverse(node => { node.castShadow = true; });
  scene.add(loaded);
  loaded.updateMatrixWorld(true);

  robot = loaded;
  robotRoot = loaded;

  const box = new Box3().setFromObject(loaded);
  const center = new Vector3();
  box.getCenter(center);
  controls.target.copy(center);
  camera.position.set(center.x + 2.0, center.y + 1.4, center.z + 2.0);
  controls.update();
}

// ── Input ─────────────────────────────────────────────────────────────────────

function setupKeyboard() {
  document.addEventListener('keydown', event => {
    if (event.code === resetKey) {
      event.preventDefault();
      const el = keyEls.get(resetKey);
      el?.classList.add('active');
      doHome().then(() => el?.classList.remove('active'));
      return;
    }
    if (event.code === switchArmKey) {
      event.preventDefault();
      const el = keyEls.get(switchArmKey);
      el?.classList.add('active');
      doSwitchArm().then(() => el?.classList.remove('active'));
      return;
    }
    if (!acceptedKeys.has(event.code)) return;
    event.preventDefault();
    activeKeys.add(event.code);
    keyEls.get(event.code)?.classList.add('active');
  });

  document.addEventListener('keyup', event => {
    if (!acceptedKeys.has(event.code)) return;
    event.preventDefault();
    activeKeys.delete(event.code);
    keyEls.get(event.code)?.classList.remove('active');
  });

  window.addEventListener('blur', () => {
    activeKeys.clear();
    keyEls.forEach(el => el.classList.remove('active'));
  });

  homeBtn.addEventListener('click', doHome);
  armBtn.addEventListener('click', doSwitchArm);
}

// ── Boot ──────────────────────────────────────────────────────────────────────

async function init() {
  statusEl.textContent = 'loading meta';
  meta = await fetchJson('/api/meta');
  applyMeta(meta);

  initScene();
  setupKeyboard();

  statusEl.textContent = 'loading urdf';
  await reloadRobot();

  const refreshMs = Number(meta.refresh_interval_ms || 40);
  setInterval(pollState, refreshMs);
  setInterval(sendKeyboard, refreshMs);

  await pollState();
  statusEl.textContent = 'ready';
  animate();
}

init().catch(err => {
  statusEl.textContent = `startup failed: ${err.message}`;
});
