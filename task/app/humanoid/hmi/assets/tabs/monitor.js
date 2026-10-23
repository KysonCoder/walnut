// 监控页：全轴表格（目标/实际/偏差/速度/力矩）、SOC 位姿表、趋势曲线、手动动作。
import { App, postCmd, pushLog, toast } from '../app.js';

let devThreshold = 0.05;   // 偏差高亮阈值（rad），页面可调
let axisOptions = [];      // [{label, logical}]

export function initMonitor() {
  const root = document.getElementById('tab-monitor');
  root.innerHTML = `
    <div class="card"><h2>全轴状态</h2>
      <div class="row" style="margin-bottom:8px">
        <label>偏差高亮阈值(rad) <input id="devTh" type="number" step="0.01"
          value="0.05" style="width:90px"/></label>
      </div>
      <div style="overflow-x:auto"><table id="axisTable"></table></div>
    </div>
    <div class="card"><h2>趋势曲线</h2>
      <div class="row" style="margin-bottom:8px">
        <select id="curveAxis"></select>
        <select id="curveMetric">
          <option value="p">实际位置</option>
          <option value="pr">目标位置</option>
          <option value="dev">偏差</option>
          <option value="v">速度</option>
          <option value="t">力矩</option>
        </select>
      </div>
      <canvas id="curveCanvas" class="curve" width="1100" height="180"></canvas>
    </div>
    <div class="card"><h2>SOC 位姿</h2>
      <div style="overflow-x:auto"><table id="poseTable"></table></div>
    </div>
    <div class="card"><h2>手动动作</h2>
      <div class="row">
        <select id="mvUnit"></select>
        <select id="mvMotionType">
          <option value="1">关节 (1)</option>
          <option value="2">直线 (2)</option>
        </select>
        <select id="mvPoseValid">
          <option value="false">pose: false</option>
          <option value="true">pose: true</option>
        </select>
      </div>
      <div class="row" style="margin-top:6px" id="mvJoints"></div>
      <div class="row" style="margin-top:6px">
        <button class="b-green" id="mvExec">执行动作</button>
      </div>
    </div>`;

  root.querySelector('#devTh').addEventListener('change', (e) => {
    const v = parseFloat(e.target.value);
    if (Number.isFinite(v) && v > 0) devThreshold = v;
  });

  // 轴选项：describe 驱动；describe 缺失时退化为 logical 序号
  axisOptions = [];
  if (App.describe && App.describe.socs.length > 0) {
    for (const soc of App.describe.socs) {
      for (const j of soc.joints) {
        axisOptions.push({ label: `${soc.name}.${j.name}`, logical: j.logical });
      }
    }
  } else {
    const n = App.describe ? App.describe.joint_count : 0;
    for (let i = 0; i < n; i++) axisOptions.push({ label: `轴${i}`, logical: i });
  }

  const sel = root.querySelector('#curveAxis');
  for (const a of axisOptions) {
    const o = document.createElement('option');
    o.value = `j.${a.logical}`;
    o.textContent = `${a.label} (logical ${a.logical})`;
    sel.appendChild(o);
  }
  sel.addEventListener('change', () => { App.curve.key = sel.value; App.curve.buf = []; });
  root.querySelector('#curveMetric').addEventListener('change', (e) => {
    App.curve.metric = e.target.value;
    App.curve.buf = [];
  });
  if (axisOptions.length > 0) { sel.value = `j.${axisOptions[0].logical}`; App.curve.key = sel.value; }

  const mvUnit = root.querySelector('#mvUnit');
  if (App.describe) {
    for (const soc of App.describe.socs) {
      const o = document.createElement('option');
      o.value = String(soc.unit_type);
      o.textContent = `${soc.name} (${soc.unit_type})`;
      mvUnit.appendChild(o);
    }
  }
  const mvJoints = root.querySelector('#mvJoints');
  for (let i = 0; i < 7; i++) {
    const inp = document.createElement('input');
    inp.id = `mvJ${i}`;
    inp.placeholder = `j${i}`;
    inp.type = 'number';
    inp.step = 'any';
    inp.style.width = '80px';
    mvJoints.appendChild(inp);
  }
  root.querySelector('#mvExec').addEventListener('click', execMovement);
}

function fmt(v) {
  return typeof v === 'number' && Number.isFinite(v) ? v.toFixed(4) : '—';
}

export function renderMonitor() {
  const s = App.state;
  if (!s || s.stale) return;

  // 全轴表
  const at = document.getElementById('axisTable');
  if (at) {
    let html = '<tr><th>轴</th><th>目标位置</th><th>实际位置</th><th>偏差</th>' +
               '<th>速度</th><th>力矩</th></tr>';
    for (const a of axisOptions) {
      const j = s.joints[a.logical];
      if (!j) continue;
      const over = Math.abs(j.dev) > devThreshold;
      html += `<tr class="${over ? 'dev-over' : ''}"><td>${a.label}</td>` +
              `<td>${fmt(j.pr)}</td><td>${fmt(j.p)}</td><td>${fmt(j.dev)}</td>` +
              `<td>${fmt(j.v)}</td><td>${fmt(j.t)}</td></tr>`;
    }
    at.innerHTML = html;
  }

  // 位姿表（pose 数组按 SOC 槽位 0..N 排列；describe.socs 跳过空单元，
  // 故按 unit_type 建名字映射而非按下标对齐）
  const pt = document.getElementById('poseTable');
  if (pt) {
    let html = '<tr><th>SOC</th><th>x</th><th>y</th><th>z</th>' +
               '<th>q0</th><th>q1</th><th>q2</th><th>q3</th></tr>';
    const nameOf = new Map();
    if (App.describe) for (const s2 of App.describe.socs) nameOf.set(s2.unit_type, s2.name);
    s.pose.forEach((p, i) => {
      html += `<tr><td>${nameOf.get(i) || `SOC${i}`}</td><td>${fmt(p.x)}</td>` +
              `<td>${fmt(p.y)}</td><td>${fmt(p.z)}</td><td>${fmt(p.q0)}</td>` +
              `<td>${fmt(p.q1)}</td><td>${fmt(p.q2)}</td><td>${fmt(p.q3)}</td></tr>`;
    });
    pt.innerHTML = html;
  }

  drawCurve();
}

function drawCurve() {
  const cv = document.getElementById('curveCanvas');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const { width: w, height: h } = cv;
  ctx.clearRect(0, 0, w, h);
  const buf = App.curve.buf;
  if (buf.length < 2) {
    ctx.fillStyle = '#9ca3af';
    ctx.font = '13px sans-serif';
    ctx.fillText('等待数据…', 12, 24);
    return;
  }
  let min = Infinity, max = -Infinity;
  for (const v of buf) { if (v < min) min = v; if (v > max) max = v; }
  if (max - min < 1e-9) { max += 0.5; min -= 0.5; }
  ctx.strokeStyle = '#e5e7eb';
  ctx.beginPath();
  ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2);
  ctx.stroke();
  ctx.strokeStyle = '#2563eb';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  buf.forEach((v, i) => {
    const x = (i / (buf.length - 1)) * w;
    const y = h - ((v - min) / (max - min)) * (h - 16) - 8;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.fillStyle = '#374151';
  ctx.font = '12px sans-serif';
  ctx.fillText(`max ${fmt(max)}`, 8, 14);
  ctx.fillText(`min ${fmt(min)}`, 8, h - 6);
}

async function execMovement() {
  const unitType = parseInt(document.getElementById('mvUnit').value, 10);
  const motionType = parseInt(document.getElementById('mvMotionType').value, 10);
  const isPoseValid = document.getElementById('mvPoseValid').value === 'true';
  const commandValue = [];
  for (let i = 0; i < 7; i++) {
    const v = parseFloat(document.getElementById(`mvJ${i}`).value);
    commandValue.push(Number.isFinite(v) ? v : 0);
  }
  pushLog(`执行动作 unitType=${unitType} motionType=${motionType}`);
  // 线上协议体与旧页一致：{cmd, program:{unitType, motionType, is_pose_valid, commandValue}}
  const r = await postCmd('/executeMovement', {
    cmd: 'FDBMANUALPROGRAM',
    program: { unitType, motionType, is_pose_valid: isPoseValid, commandValue },
  });
  const info = r.body && r.body.info ? ` (${r.body.info})` : '';
  pushLog(`executeMovement → HTTP ${r.status}${info}`);
  if (!r.ok) toast(`执行动作被拒: HTTP ${r.status}${info}`, true);
}
