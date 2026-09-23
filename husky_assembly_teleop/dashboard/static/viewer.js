// Three.js viewer for one derivation attempt.
//
// The scene geometry is baked once per design problem (scene.glb); everything
// that moves arrives as world matrices from the server, so this file only
// assigns matrices, recolours the links that collide, and drives the camera.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

// Same pair the PyBullet-side collision diagnosis uses, so the two views agree.
const HIT_A = 0xff8c00;
const HIT_B = 0x00e5ff;
const GHOST = { home_bar: 0x37c871, goal_bar: 0x4f9df7, break_bar: 0xe4574f };

export function createViewer(host) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  host.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x14161a);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 200);
  camera.up.set(0, 0, 1);                       // the cell is Z-up
  camera.position.set(3.2, -3.2, 2.4);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.screenSpacePanning = true;

  scene.add(new THREE.HemisphereLight(0xdfe6f0, 0x20242c, 1.1));
  const key = new THREE.DirectionalLight(0xffffff, 1.5);
  key.position.set(4, -5, 6);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xffffff, 0.5);
  fill.position.set(-5, 4, 3);
  scene.add(fill);
  scene.add(new THREE.GridHelper(20, 20, 0x39404b, 0x262b33).rotateX(Math.PI / 2));

  let root = null;                 // the loaded glTF scene
  let nodeByName = new Map();      // node name -> Object3D
  let baseMaterials = new Map();   // node name -> cloned materials (recolourable)
  let baseColours = new Map();     // node name -> original colours
  let highlighted = [];
  let ghostGroup = new THREE.Group();
  let contactGroup = new THREE.Group();
  scene.add(ghostGroup, contactGroup);
  let animated = [];               // Object3D per frame-matrix column
  let frames = [];
  let loadedGlb = null;
  let hiddenNodes = [];

  function resize() {
    const w = host.clientWidth || 1;
    const h = host.clientHeight || 1;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(host);
  renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });

  // Clone every material once so recolouring one node never bleeds into the
  // other nodes that share an exported material.
  function indexNodes() {
    nodeByName = new Map();
    baseMaterials = new Map();
    baseColours = new Map();
    root.traverse((object) => {
      if (!object.name || !object.name.includes('__')) return;
      nodeByName.set(object.name, object);
      const materials = [];
      const colours = [];
      object.traverse((child) => {
        if (!child.isMesh) return;
        child.material = child.material.clone();
        materials.push(child.material);
        colours.push(child.material.color.getHex());
      });
      baseMaterials.set(object.name, materials);
      baseColours.set(object.name, colours);
    });
  }

  function clearHighlight() {
    highlighted.forEach((name) => {
      const materials = baseMaterials.get(name) || [];
      const colours = baseColours.get(name) || [];
      materials.forEach((material, i) => {
        material.color.setHex(colours[i]);
        material.emissive.setHex(0x000000);
      });
    });
    highlighted = [];
    contactGroup.clear();
  }

  function highlight(names, collisions) {
    clearHighlight();
    (names || []).forEach((name, index) => {
      const materials = baseMaterials.get(name);
      if (!materials) return;
      const colour = index % 2 === 0 ? HIT_A : HIT_B;
      materials.forEach((material) => {
        material.color.setHex(colour);
        material.emissive.setHex(colour);
        material.emissiveIntensity = 0.35;
      });
      highlighted.push(name);
    });
    (collisions || []).forEach((hit) => {
      if (!hit.point) return;
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.018, 12, 12),
        new THREE.MeshBasicMaterial({ color: 0xffffff }));
      marker.position.fromArray(hit.point);
      contactGroup.add(marker);
    });
  }

  function loadScene(glbUrl, staticBodies) {
    return new Promise((resolve, reject) => {
      if (loadedGlb === glbUrl) { applyStatic(staticBodies); resolve(); return; }
      new GLTFLoader().load(glbUrl, (gltf) => {
        if (root) scene.remove(root);
        root = gltf.scene;
        scene.add(root);
        loadedGlb = glbUrl;
        indexNodes();
        root.traverse((object) => { object.matrixAutoUpdate = false; });
        applyStatic(staticBodies);
        resolve();
      }, undefined, reject);
    });
  }

  // Bodies that never move during a derivation get their pose once. The ones
  // the planner was told to ignore are drawn faintly (and can be hidden).
  function applyStatic(staticBodies) {
    hiddenNodes = [];
    (staticBodies || []).forEach((body) => {
      const object = nodeByName.get(body.node);
      if (!object) return;
      object.matrix.identity();
      const q = body.world.quat_xyzw;
      object.matrix.compose(
        new THREE.Vector3().fromArray(body.world.pos),
        new THREE.Quaternion(q[0], q[1], q[2], q[3]),
        new THREE.Vector3(1, 1, 1));
      object.matrixWorldNeedsUpdate = true;
      if (body.hidden) {
        hiddenNodes.push(object);
        object.visible = false;
      }
    });
    root.updateMatrixWorld(true);
  }

  function setHiddenVisible(visible) {
    hiddenNodes.forEach((object) => {
      object.visible = visible;
      object.traverse((child) => {
        if (!child.isMesh) return;
        child.material.transparent = visible;
        child.material.opacity = visible ? 0.18 : 1.0;
      });
    });
  }

  function showCandidate(payload) {
    frames = payload.frames || [];
    animated = (payload.nodes || []).map((name) => nodeByName.get(name) || null);
    ghostGroup.clear();
    const barNode = nodeByName.get(payload.active_bar_node);
    Object.entries(payload.ghosts || {}).forEach(([which, matrix]) => {
      if (!matrix || !barNode) return;
      const ghost = barNode.clone(true);
      ghost.traverse((child) => {
        if (!child.isMesh) return;
        child.material = child.material.clone();
        child.material.transparent = true;
        child.material.opacity = 0.3;
        child.material.color.setHex(GHOST[which] || 0xffffff);
        child.material.depthWrite = false;
      });
      ghost.matrixAutoUpdate = false;
      ghost.matrix.fromArray(matrix);
      ghost.matrixWorldNeedsUpdate = true;
      ghost.visible = true;
      ghostGroup.add(ghost);
    });
    setFrame(0, payload);
    resetView();
    return frames.length;
  }

  function setFrame(index, payload) {
    const frame = frames[Math.max(0, Math.min(frames.length - 1, index))];
    if (!frame) return null;
    animated.forEach((object, i) => {
      if (!object) return;
      object.matrix.fromArray(frame.matrices[i]);
      object.matrixWorldNeedsUpdate = true;
    });
    if (root) root.updateMatrixWorld(true);
    const collisions = (payload && payload.collisionsFor)
      ? payload.collisionsFor(index) : null;
    highlight(frame.colliding, collisions);
    return frame.label;
  }

  function resetView() {
    if (!root) return;
    const box = new THREE.Box3();
    nodeByName.forEach((object, name) => {
      if (name.startsWith('robot__') || name.startsWith('tool__')) box.expandByObject(object);
    });
    ghostGroup.children.forEach((ghost) => box.expandByObject(ghost));
    if (box.isEmpty()) return;
    const centre = box.getCenter(new THREE.Vector3());
    const radius = Math.max(box.getSize(new THREE.Vector3()).length() * 0.6, 1.0);
    controls.target.copy(centre);
    camera.position.copy(centre).add(new THREE.Vector3(radius, -radius, radius * 0.75));
    camera.near = radius / 100;
    camera.far = radius * 40;
    camera.updateProjectionMatrix();
    controls.update();
  }

  resize();
  return { loadScene, showCandidate, setFrame, resetView, setHiddenVisible };
}
