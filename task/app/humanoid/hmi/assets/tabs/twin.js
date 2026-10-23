// 3D 孪生页：URDF 解析 + three.js 场景 + 反馈关节角 FK 驱动。
// mesh 加载失败的 link 降级为骨架（关节球 + 连杆线）；three 库缺失时整页降级提示。
// 场景推迟到页签首次可见才创建：隐藏时容器 clientWidth=0，会建出 0×0 画布永久空白
//（2026-09-30 3D 页不显示根因）。
import { App, api } from '../app.js';

// URDF 关节名 → (SOC 名, 序号偏移)。第三元素：有捕获组时加到捕获序号上
//（如 wrist_RJ1 → 1+5=6 → J6）；无捕获组时作为绝对序号（如 base_J → J1）。
// 显示模型来自 meshes_src/<flavor>/ 显示包（dual_handarm_0906 / NieNieDual_0829），
// 与 cfg/ 控制 URDF 命名体系不同。显示包没有的关节（humanoid 腰/头）不映射，恒 0。
const URDF_JOINT_RULES = {
  humanoid: [
    [/^Arm_RJ(\d+)$/, 'RIGHT_ARM', 0],
    [/^wrist_RJ(\d+)$/, 'RIGHT_ARM', 5],
    [/^Arm_LJ(\d+)$/, 'LEFT_ARM', 0],
    [/^wrist_LJ(\d+)$/, 'LEFT_ARM', 5],
    [/^RTB_J(\d+)$/, 'RIGHT_THUMB', 0],
    [/^RIF_J(\d+)$/, 'RIGHT_FF', 0],
    [/^RMF_J(\d+)$/, 'RIGHT_MF', 0],
    [/^RRF_J(\d+)$/, 'RIGHT_RF', 0],
    [/^RLF_J(\d+)$/, 'RIGHT_LF', 0],
    [/^LTB_J(\d+)$/, 'LEFT_THUMB', 0],
    [/^LIF_J(\d+)$/, 'LEFT_FF', 0],
    [/^LMF_J(\d+)$/, 'LEFT_MF', 0],
    [/^LRF_J(\d+)$/, 'LEFT_RF', 0],
    [/^LLF_J(\d+)$/, 'LEFT_LF', 0],
  ],
  dualarm: [
    [/^base_J$/, 'ROBOT_BODY', 1],
    [/^LJ(\d+)$/, 'LEFT_ARM', 0],
    [/^RJ(\d+)$/, 'RIGHT_ARM', 0],
  ],
};

let T = null;              // three 命名空间（动态 import）
let renderer = null, scene = null, camera = null, controls = null;
let grid = null;
let movableJoints = [];    // {node, baseQ, axis, logical}
let meshGhost = false;
let stlLoader = null;
let meshStats = { total: 0, settled: 0, failed: 0 };
let twHome = null, twTarget = null;   // 视角复位位姿（建模后按包围盒设定）
let starting = false;      // 场景异步初始化进行中

function logicalForJoint(urdfName) {
  if (!App.describe) return -1;
  const rules = URDF_JOINT_RULES[App.describe.flavor] || [];
  for (const [re, socName, offset] of rules) {
    const m = urdfName.match(re);
    if (!m) continue;
    const idx = m[1] !== undefined ? parseInt(m[1], 10) + offset : offset;
    const soc = App.describe.socs.find((s) => s.name === socName);
    if (!soc || idx < 1) return -1;
    const j = soc.joints[idx - 1];
    return j ? j.logical : -1;
  }
  return -1;
}

function parse3(s, dflt) {
  const v = (s || '').trim().split(/\s+/).map(Number);
  return [0, 1, 2].map((i) => (Number.isFinite(v[i]) ? v[i] : dflt[i]));
}

function parseUrdf(xmlText) {
  const doc = new DOMParser().parseFromString(xmlText, 'text/xml');
  const links = {};
  doc.querySelectorAll('link').forEach((ln) => {
    const meshes = [];
    ln.querySelectorAll('visual').forEach((v) => {
      const mesh = v.querySelector('mesh');
      if (!mesh || !mesh.getAttribute('filename')) return;
      const o = v.querySelector('origin');
      meshes.push({
        file: mesh.getAttribute('filename'),
        xyz: o ? parse3(o.getAttribute('xyz'), [0, 0, 0]) : [0, 0, 0],
        rpy: o ? parse3(o.getAttribute('rpy'), [0, 0, 0]) : [0, 0, 0],
        scale: parse3(mesh.getAttribute('scale'), [1, 1, 1]),
      });
    });
    links[ln.getAttribute('name')] = { meshes };
  });
  const joints = [];
  doc.querySelectorAll('joint').forEach((j) => {
    const parent = j.querySelector('parent');
    const child = j.querySelector('child');
    if (!parent || !child) return;
    const o = j.querySelector('origin');
    const ax = j.querySelector('axis');
    joints.push({
      name: j.getAttribute('name'),
      type: j.getAttribute('type'),
      parent: parent.getAttribute('link'),
      child: child.getAttribute('link'),
      xyz: o ? parse3(o.getAttribute('xyz'), [0, 0, 0]) : [0, 0, 0],
      rpy: o ? parse3(o.getAttribute('rpy'), [0, 0, 0]) : [0, 0, 0],
      axis: ax ? parse3(ax.getAttribute('xyz'), [1, 0, 0]) : [1, 0, 0],
    });
  });
  return { links, joints };
}

function meshUrl(file) {
  return file.replace(/^package:\/\/[^/]+\/meshes\//, '/meshes/');
}

function updateMeshStatus() {
  const el = document.getElementById('twStatus');
  if (!el) return;
  const base = el.dataset.base || '';
  el.textContent = meshStats.total > 0
    ? `${base} · mesh ${meshStats.settled}/${meshStats.total}` +
      (meshStats.failed > 0 ? `（${meshStats.failed} 个失败转骨架）` : '')
    : base;
}

function addSkeleton(linkGroup, joint) {
  // 降级：关节球 + 从父 link 原点到本关节原点的连杆线
  const sphere = new T.Mesh(
    new T.SphereGeometry(0.02, 12, 8),
    new T.MeshStandardMaterial({ color: 0x3b82f6 }));
  sphere.position.set(joint.xyz[0], joint.xyz[1], joint.xyz[2]);
  linkGroup.add(sphere);
  const geo = new T.BufferGeometry().setFromPoints([
    new T.Vector3(0, 0, 0),
    new T.Vector3(joint.xyz[0], joint.xyz[1], joint.xyz[2]),
  ]);
  linkGroup.add(new T.Line(geo, new T.LineBasicMaterial({ color: 0x6b7280 })));
}

// 给一个 link 挂 mesh；skelJoint 非空时 mesh 失败/缺声明 → 骨架降级
function attachMeshes(linkGroup, meshes, skelParent, skelJoint) {
  if (meshes.length === 0) {
    if (skelJoint) addSkeleton(skelParent, skelJoint);
    return;
  }
  for (const m of meshes) {
    meshStats.total++;
    stlLoader.load(meshUrl(m.file), (geom) => {
      const mat = new T.MeshStandardMaterial({ color: 0x9ca3af, metalness: 0.1 });
      const mesh = new T.Mesh(geom, mat);
      mesh.position.set(m.xyz[0], m.xyz[1], m.xyz[2]);
      mesh.rotation.set(m.rpy[0], m.rpy[1], m.rpy[2], 'ZYX');
      mesh.scale.set(m.scale[0], m.scale[1], m.scale[2]);
      linkGroup.add(mesh);
      meshStats.settled++;
      updateMeshStatus();
    }, undefined, () => {
      meshStats.settled++;
      meshStats.failed++;
      if (skelJoint) addSkeleton(skelParent, skelJoint);
      updateMeshStatus();
    });
  }
}

function buildRobot(model) {
  const robot = new T.Group();
  const linkGroups = {};
  for (const name of Object.keys(model.links)) {
    const g = new T.Group();
    g.name = name;
    linkGroups[name] = g;
  }
  const isChild = new Set(model.joints.map((j) => j.child));
  // 根 link 先入树并挂 mesh（根没有父关节，joints 循环覆盖不到——
  // 2026-09-30 修复前基座/躯干 mesh 永远不加载）
  for (const name of Object.keys(model.links)) {
    if (!isChild.has(name)) {
      robot.add(linkGroups[name]);
      attachMeshes(linkGroups[name], model.links[name].meshes,
                   linkGroups[name], { xyz: [0, 0, 0] });
    }
  }
  for (const j of model.joints) {
    const parent = linkGroups[j.parent];
    const child = linkGroups[j.child];
    if (!parent || !child) continue;
    const jg = new T.Group();
    jg.position.set(j.xyz[0], j.xyz[1], j.xyz[2]);
    // URDF 固定轴 RPY = three.js Euler(roll, pitch, yaw, 'ZYX')
    jg.rotation.set(j.rpy[0], j.rpy[1], j.rpy[2], 'ZYX');
    parent.add(jg);
    jg.add(child);
    if (j.type === 'revolute' || j.type === 'continuous') {
      movableJoints.push({
        node: jg,
        baseQ: jg.quaternion.clone(),
        axis: new T.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize(),
        logical: logicalForJoint(j.name),
      });
    }
    attachMeshes(child, model.links[j.child].meshes, parent, j);
  }
  return robot;
}

function syncSize() {
  if (!renderer || !camera) return;
  const wrap = document.getElementById('twCanvasWrap');
  if (!wrap) return;
  const W = wrap.clientWidth, H = wrap.clientHeight;
  if (W === 0 || H === 0) return;   // 页签仍隐藏，下帧再试
  const cur = new T.Vector2();
  renderer.getSize(cur);
  if (cur.x !== W || cur.y !== H) {
    renderer.setSize(W, H);
    camera.aspect = W / H;
    camera.updateProjectionMatrix();
  }
}

function animate() {
  requestAnimationFrame(animate);
  const section = document.getElementById('tab-twin');
  if (!section || section.classList.contains('hidden')) return;  // 页签不可见不渲染
  syncSize();
  const s = App.state;
  if (s && !s.stale) {
    const q = new T.Quaternion();
    for (const j of movableJoints) {
      if (j.logical < 0) continue;
      const fb = s.joints[j.logical];
      if (!fb) continue;
      q.setFromAxisAngle(j.axis, fb.p);
      j.node.quaternion.copy(j.baseQ).multiply(q);
    }
  }
  controls.update();
  renderer.render(scene, camera);
}

export function initTwin() {
  // 只建外壳（幂等）；three 场景推迟到页签首次可见（renderTwin → startScene）
  if (document.getElementById('twCanvasWrap')) return;
  const root = document.getElementById('tab-twin');
  root.innerHTML = `
    <div class="card"><h2>3D 数字孪生</h2>
      <div class="row" style="margin-bottom:8px">
        <button class="b-gray b-sm" id="twGrid">网格开/关</button>
        <button class="b-gray b-sm" id="twGhost">透明/实体</button>
        <button class="b-gray b-sm" id="twHome">视角复位</button>
        <span id="twStatus" style="font-size:12px;color:#6b7280"></span>
      </div>
      <div id="twCanvasWrap" style="height:560px;background:#111827;border-radius:8px"></div>
    </div>`;
}

async function startScene() {
  const status = document.getElementById('twStatus');
  try {
    await startSceneInner(status);
  } catch (e) {
    // 异步初始化失败必须显式可见（静默 rejection = 空白页无提示）
    status.textContent = `初始化失败: ${e.message || e}`;
  }
}

async function startSceneInner(status) {
  try {
    const three = await import('three');
    const [{ OrbitControls }, { STLLoader }] = await Promise.all([
      import('/vendor/OrbitControls.js'),
      import('/vendor/STLLoader.js'),
    ]);
    T = Object.assign({}, three, { OrbitControls });
    stlLoader = new STLLoader();
  } catch (e) {
    status.textContent = `3D 库加载失败（vendor 文件缺失？）: ${e.message}`;
    return;
  }
  if (!App.describe) {
    status.textContent = '机器人描述不可用，3D 页停用';
    return;
  }
  const r = await api(App.describe.urdf);
  if (!r.ok) {
    status.textContent = `URDF 加载失败: HTTP ${r.status}`;
    return;
  }
  const wrap = document.getElementById('twCanvasWrap');
  renderer = new T.WebGLRenderer({ antialias: true });
  wrap.appendChild(renderer.domElement);
  syncSize();   // 此刻页签可见，拿到真实尺寸（隐藏时 clientWidth=0）
  scene = new T.Scene();
  scene.background = new T.Color(0x111827);
  camera = new T.PerspectiveCamera(50, 16 / 9, 0.01, 100);
  camera.up.set(0, 0, 1);   // URDF 是 Z-up 世界；相机 up 对齐后模型才是立姿
  camera.position.set(1.6, -1.6, 1.2);
  syncSize();   // 修正 camera.aspect 为真实宽高比
  controls = new T.OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.5);
  scene.add(new T.AmbientLight(0xffffff, 0.7));
  const dir = new T.DirectionalLight(0xffffff, 1.2);
  dir.position.set(2, -2, 3);
  scene.add(dir);
  grid = new T.GridHelper(4, 20, 0x374151, 0x1f2937);
  grid.rotation.x = Math.PI / 2;   // Z-up 世界
  scene.add(grid);

  const model = parseUrdf(r.bodyText || '');
  const robot = buildRobot(model);
  scene.add(robot);
  // 按连杆世界位置取包围盒定初始视角：显示包原点不在几何中心
  //（dual_handarm 原点在躯干、手臂下垂到 z≈-0.86，固定 target 会把模型甩出画面）
  robot.updateMatrixWorld(true);
  const bb = new T.Box3();
  const wp = new T.Vector3();
  robot.traverse((o) => { if (o !== robot) { o.getWorldPosition(wp); bb.expandByPoint(wp); } });
  const center = bb.getCenter(new T.Vector3());
  const size = bb.getSize(new T.Vector3()).length();
  const dist = Math.max(size, 0.5) * 1.2;
  const homePos = center.clone().add(new T.Vector3(dist, -dist, dist * 0.6));
  camera.position.copy(homePos);
  controls.target.copy(center);
  grid.position.set(center.x, center.y, bb.min.z);   // 地面网格贴模型底部
  twHome = homePos; twTarget = center.clone();
  status.dataset.base =
    `${App.describe.flavor} · 可动关节 ${movableJoints.length}`;
  window.__twin = { scene, movableJoints };   // 诊断钩子（marionette/控制台用）
  updateMeshStatus();

  document.getElementById('twGrid').onclick = () => { grid.visible = !grid.visible; };
  document.getElementById('twGhost').onclick = () => {
    meshGhost = !meshGhost;
    scene.traverse((o) => {
      if (o.isMesh && o.material && o.geometry &&
          o.geometry.type !== 'SphereGeometry') {
        o.material.transparent = meshGhost;
        o.material.opacity = meshGhost ? 0.35 : 1.0;
      }
    });
  };
  document.getElementById('twHome').onclick = () => {
    if (twHome && twTarget) {
      camera.position.copy(twHome);
      controls.target.copy(twTarget);
    }
  };
  window.addEventListener('resize', syncSize);
  animate();
}

export function renderTwin() {
  // 页签激活时（含 10Hz 轮询）经此进入；场景只初始化一次
  if (!document.getElementById('twCanvasWrap')) initTwin();
  if (renderer || starting) { syncSize(); return; }
  starting = true;
  startScene().finally(() => { starting = false; });
}
