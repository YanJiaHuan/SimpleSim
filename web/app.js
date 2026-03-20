import {
  AmbientLight,
  Box3,
  Color,
  DirectionalLight,
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

const viewer = document.getElementById('viewer');
const statusEl = document.getElementById('status');
const eePoseEl = document.getElementById('ee-pose');
const ikStateEl = document.getElementById('ik-state');
const robotNameEl = document.getElementById('robot-name');

const activeKeys = new Set();
const acceptedKeys = new Set([
  'KeyW',
  'KeyS',
  'KeyA',
  'KeyD',
  'KeyR',
  'KeyF',
  'KeyQ',
  'KeyE',
  'ArrowUp',
  'ArrowDown',
  'ArrowLeft',
  'ArrowRight',
]);

let meta = null;
let robot = null;
let renderer = null;
let scene = null;
let camera = null;
let controls = null;
let busy = false;

function formatPose(pose) {
  if (!pose || pose.length !== 6) {
    return '[]';
  }
  return JSON.stringify(pose.map(v => Number(v).toFixed(4)), null, 2);
}

function applyState(state) {
  if (!state) {
    return;
  }

  if (state.ee_pose) {
    eePoseEl.textContent = formatPose(state.ee_pose);
  }

  if (state.ik) {
    ikStateEl.textContent = JSON.stringify(state.ik, null, 2);
  }

  if (!robot || !state.joint_names || !state.q) {
    return;
  }

  const joints = {};
  for (let i = 0; i < state.joint_names.length; i += 1) {
    joints[state.joint_names[i]] = state.q[i];
  }
  robot.setJointValues(joints);
  robot.updateMatrixWorld(true);
}

async function fetchJson(url, options = {}) {
  const response = await fetch(url, options);
  if (!response.ok) {
    throw new Error(`${url} -> ${response.status}`);
  }
  return response.json();
}

async function pollState() {
  try {
    const state = await fetchJson('/api/state');
    applyState(state);
  } catch (err) {
    statusEl.textContent = `state poll failed: ${err.message}`;
  }
}

async function sendKeyboard() {
  if (busy || activeKeys.size === 0) {
    return;
  }

  busy = true;
  try {
    const payload = { keys: Array.from(activeKeys) };
    const state = await fetchJson('/api/keyboard', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    statusEl.textContent = state.success ? 'state: running' : 'state: ik failed';
    applyState(state);
  } catch (err) {
    statusEl.textContent = `keyboard failed: ${err.message}`;
  } finally {
    busy = false;
  }
}

function initScene() {
  scene = new Scene();
  scene.background = new Color(0xe2e8f0);

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

  const ground = new Mesh(new PlaneGeometry(12, 12), new ShadowMaterial({ opacity: 0.25 }));
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = 0;
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

async function loadRobot() {
  const loader = new URDFLoader();
  robot = await new Promise((resolve, reject) => {
    loader.load(
      meta.urdf_url,
      loaded => resolve(loaded),
      undefined,
      err => reject(err),
    );
  });

  robot.traverse(node => {
    node.castShadow = true;
  });

  scene.add(robot);

  robot.updateMatrixWorld(true);
  const box = new Box3().setFromObject(robot);
  const center = new Vector3();
  box.getCenter(center);
  controls.target.copy(center);
  camera.position.set(center.x + 2.0, center.y + 1.4, center.z + 2.0);
  controls.update();
}

function setupKeyboard() {
  document.addEventListener('keydown', event => {
    if (!acceptedKeys.has(event.code)) {
      return;
    }
    event.preventDefault();
    activeKeys.add(event.code);
  });

  document.addEventListener('keyup', event => {
    if (!acceptedKeys.has(event.code)) {
      return;
    }
    event.preventDefault();
    activeKeys.delete(event.code);
  });

  window.addEventListener('blur', () => activeKeys.clear());
}

async function init() {
  statusEl.textContent = 'state: loading meta';
  meta = await fetchJson('/api/meta');
  robotNameEl.textContent = `${meta.robot_name} | ${meta.base_link} -> ${meta.ee_link}`;

  initScene();
  setupKeyboard();

  statusEl.textContent = 'state: loading urdf';
  await loadRobot();

  const refreshMs = Number(meta.refresh_interval_ms || 40);
  setInterval(pollState, refreshMs);
  setInterval(sendKeyboard, refreshMs);

  await pollState();
  statusEl.textContent = 'state: ready';
  animate();
}

init().catch(err => {
  statusEl.textContent = `startup failed: ${err.message}`;
});

