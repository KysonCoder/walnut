// 示教页：目标点列表（记录当前/编辑/排序/删除）、执行、导入导出、后端持久化。
// 点位结构：{name, soc(unit_type int), type:"joint"|"pose", values:[7 个数]}
import { App, api, postCmd, pushLog, toast } from '../app.js';

const $ = (id) => document.getElementById(id);

function finite7(arr) {
  return Array.isArray(arr) && arr.length === 7 && arr.every((v) => Number.isFinite(v));
}

export async function loadTeachPoints() {
  const r = await api('/api/teach/points');
  if (r.ok && r.body && Array.isArray(r.body.points)) {
    App.teachPoints = r.body.points.filter(
      (p) => p && typeof p.name === 'string' && finite7(p.values));
    if (r.body.warning) toast(r.body.warning, true);
    renderTeach();
  } else if (!r.ok) {
    toast(`点位加载失败: HTTP ${r.status}`, true);
  }
}

async function saveTeachPoints() {
  // 提交前整体有限数校验（双防线的前端侧）
  for (const p of App.teachPoints) {
    if (!finite7(p.values)) { toast(`点位 ${p.name} 含非法数值，未保存`, true); return; }
  }
  const r = await postCmd('/api/teach/points', { points: App.teachPoints });
  if (r.ok) { pushLog(`点位已保存（${r.body.count} 条）`); toast('点位已保存'); }
  else toast(`保存失败: ${r.body && r.body.info ? r.body.info : 'HTTP ' + r.status}`, true);
}

function socOptions(selected) {
  const socs = App.describe ? App.describe.socs : [];
  return socs.map((s) =>
    `<option value="${s.unit_type}" ${s.unit_type === selected ? 'selected' : ''}>` +
    `${s.name} (${s.unit_type})</option>`).join('');
}

function recordCurrent(idx) {
  const p = App.teachPoints[idx];
  const s = App.state;
  if (!s || s.stale) { toast('反馈不可用，无法记录当前位置', true); return; }
  if (p.type === 'joint') {
    const soc = App.describe && App.describe.socs.find((x) => x.unit_type === p.soc);
    if (!soc) { toast('该 SOC 不在 describe 中', true); return; }
    const vals = [0, 0, 0, 0, 0, 0, 0];
    soc.joints.forEach((j, i) => {
      if (i < 7 && s.joints[j.logical]) vals[i] = s.joints[j.logical].p;
    });
    p.values = vals;
  } else {
    const pose = s.pose[p.soc];
    if (!pose) { toast('该 SOC 位姿不可用', true); return; }
    p.values = [pose.x, pose.y, pose.z, pose.q0, pose.q1, pose.q2, pose.q3];
  }
  pushLog(`点位 ${p.name} 已记录当前${p.type === 'joint' ? '关节' : '位姿'}`);
  renderTeach();
}

async function executePoint(idx) {
  const p = App.teachPoints[idx];
  if (!finite7(p.values)) { toast('点位含非法数值，拒绝执行', true); return; }
  const joint = p.type === 'joint';
  const r = await postCmd('/executeMovement', {
    cmd: 'FDBMANUALPROGRAM',
    program: {
      unitType: p.soc,
      motionType: joint ? 1 : 2,
      is_pose_valid: !joint,
      commandValue: p.values,
    },
  });
  const info = r.body && r.body.info ? ` (${r.body.info})` : '';
  pushLog(`示教点 ${p.name} 执行 → HTTP ${r.status}${info}`);
  if (!r.ok) toast(`执行被拒: HTTP ${r.status}${info}`, true);
}

export function initTeach() {
  const root = $('tab-teach');
  root.innerHTML = `
    <div class="card"><h2>目标点</h2>
      <div class="row" style="margin-bottom:8px">
        <button class="b-green" id="tpAdd">新增点位</button>
        <button class="b-blue" id="tpSave">保存到后端</button>
        <button class="b-gray" id="tpReload">重新加载</button>
        <button class="b-purple" id="tpExport">导出 JSON</button>
        <label class="b-gray" style="padding:8px 14px;border-radius:8px;cursor:pointer">
          导入 JSON<input type="file" id="tpImport" accept=".json" class="hidden"/></label>
      </div>
      <div id="tpList"></div>
    </div>`;
  $('tpAdd').addEventListener('click', () => {
    const firstSoc = App.describe && App.describe.socs.length
      ? App.describe.socs[0].unit_type : 0;
    App.teachPoints.push({
      name: `P${App.teachPoints.length + 1}`,
      soc: firstSoc, type: 'joint', values: [0, 0, 0, 0, 0, 0, 0],
    });
    renderTeach();
  });
  $('tpSave').addEventListener('click', saveTeachPoints);
  $('tpReload').addEventListener('click', loadTeachPoints);
  $('tpExport').addEventListener('click', () => {
    const blob = new Blob([JSON.stringify({ points: App.teachPoints }, null, 2)],
      { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'teach_points.json';
    a.click();
    URL.revokeObjectURL(a.href);
  });
  $('tpImport').addEventListener('change', async (e) => {
    const f = e.target.files[0];
    if (!f) return;
    try {
      const doc = JSON.parse(await f.text());
      if (!doc || !Array.isArray(doc.points)) throw new Error('缺少 points 数组');
      for (const p of doc.points) {
        if (!p || typeof p.name !== 'string' || !finite7(p.values) ||
            !Number.isInteger(p.soc) || !['joint', 'pose'].includes(p.type)) {
          throw new Error(`非法点位: ${p && p.name ? p.name : '(无名)'}`);
        }
      }
      App.teachPoints = doc.points;
      pushLog(`导入点位 ${doc.points.length} 条`);
      renderTeach();
    } catch (err) {
      toast(`导入失败: ${err.message}`, true);
    }
    e.target.value = '';
  });
}

export function renderTeach() {
  const list = $('tpList');
  if (!list) return;
  if (App.teachPoints.length === 0) {
    list.innerHTML = '<div style="color:#9ca3af">暂无目标点</div>';
    return;
  }
  list.innerHTML = '';
  App.teachPoints.forEach((p, idx) => {
    const card = document.createElement('div');
    card.className = 'group';
    card.innerHTML = `
      <div class="group-head">
        <input data-f="name" value="${p.name}" style="width:110px"/>
        <select data-f="soc">${socOptions(p.soc)}</select>
        <select data-f="type">
          <option value="joint" ${p.type === 'joint' ? 'selected' : ''}>关节</option>
          <option value="pose" ${p.type === 'pose' ? 'selected' : ''}>位姿</option>
        </select>
        <span style="flex:1"></span>
        <button class="b-green b-sm" data-act="record">记录当前</button>
        <button class="b-blue b-sm" data-act="exec">执行</button>
        <button class="b-gray b-sm" data-act="up" ${idx === 0 ? 'disabled' : ''}>↑</button>
        <button class="b-gray b-sm" data-act="down"
          ${idx === App.teachPoints.length - 1 ? 'disabled' : ''}>↓</button>
        <button class="b-red b-sm" data-act="del">删除</button>
      </div>
      <div class="group-body"><div class="row cvrow">
        ${p.values.map((v, i) =>
          `<input data-v="${i}" type="number" step="any" value="${v}"
             style="width:90px" placeholder="v${i}"/>`).join('')}
      </div></div>`;
    card.querySelectorAll('[data-f]').forEach((inp) => {
      inp.addEventListener('change', () => {
        const f = inp.dataset.f;
        if (f === 'soc') p.soc = parseInt(inp.value, 10);
        else if (f === 'name') p.name = inp.value.slice(0, 64);
        else p.type = inp.value;
      });
    });
    card.querySelectorAll('[data-v]').forEach((inp) => {
      inp.addEventListener('change', () => {
        const v = parseFloat(inp.value);
        if (Number.isFinite(v)) p.values[+inp.dataset.v] = v;
        else { inp.value = p.values[+inp.dataset.v]; toast('非法数值已回退', true); }
      });
    });
    card.querySelectorAll('[data-act]').forEach((btn) => {
      btn.addEventListener('click', () => {
        const act = btn.dataset.act;
        if (act === 'record') recordCurrent(idx);
        else if (act === 'exec') executePoint(idx);
        else if (act === 'del') { App.teachPoints.splice(idx, 1); renderTeach(); }
        else if (act === 'up' && idx > 0) {
          [App.teachPoints[idx - 1], App.teachPoints[idx]] =
            [App.teachPoints[idx], App.teachPoints[idx - 1]];
          renderTeach();
        } else if (act === 'down' && idx < App.teachPoints.length - 1) {
          [App.teachPoints[idx + 1], App.teachPoints[idx]] =
            [App.teachPoints[idx], App.teachPoints[idx + 1]];
          renderTeach();
        }
      });
    });
    list.appendChild(card);
  });
}
