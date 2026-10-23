// 程序页：可视化程序组构建器（迁移自旧 robot_hmi.html）+ JSON 文本预览 + 循环执行 +
// 停止 + 后端程序存取 + 示教点引用填充。
import { App, api, postCmd, pushLog, toast } from '../app.js';

const $ = (sel) => document.getElementById('tab-program').querySelector(sel);

const programState = { name: '', cmd: 'TESTAUTOPROGRAM', actionGroup: [] };

function newEmptyAction() {
  return {
    name: '', unitType: 0, motionType: 1, is_pose_valid: false,
    speed: { tcp_m_s: 10, ori_rad_s: 10, joint_rad_s: 10, ext_rad_s: 10 },
    commandValue: [0, 0, 0, 0, 0, 0, 0],
    currentActionGroup: 1,
  };
}

function swap(arr, i, j) { const t = arr[i]; arr[i] = arr[j]; arr[j] = t; }

function validateProgram(state) {
  const errs = [];
  if (!state.cmd) errs.push('cmd 不能为空');
  if (!Array.isArray(state.actionGroup) || !state.actionGroup.length) {
    errs.push('至少需要 1 个动作组');
  }
  state.actionGroup.forEach((group, g) => {
    if (!group.length) errs.push(`动作组 ${g + 1} 为空`);
    group.forEach((a, ai) => {
      if (!Array.isArray(a.commandValue) || a.commandValue.length !== 7 ||
          !a.commandValue.every((v) => Number.isFinite(v))) {
        errs.push(`组${g + 1} 动作${ai + 1} commandValue 需为 7 个有限数字`);
      }
    });
  });
  return errs;
}

function teachPointOptions() {
  return ['<option value="">引用示教点…</option>'].concat(
    App.teachPoints.map((p, i) =>
      `<option value="${i}">${p.name} (${p.type})</option>`)).join('');
}

function applyTeachPoint(action, idx) {
  const p = App.teachPoints[idx];
  if (!p) return;
  action.unitType = p.soc;
  action.motionType = p.type === 'joint' ? 1 : 2;
  action.is_pose_valid = p.type !== 'joint';
  action.commandValue = p.values.slice();
  action.name = p.name;
}

function actionRowHTML(action) {
  const cv = action.commandValue || [];
  const speed = action.speed || { tcp_m_s: '', ori_rad_s: '', joint_rad_s: '', ext_rad_s: '' };
  return `
  <div>
    <label>unitType <input data-field="unitType" type="number"
      value="${action.unitType ?? ''}" style="width:70px"/></label>
    <label>motionType <input data-field="motionType" type="number"
      value="${action.motionType ?? ''}" style="width:80px"/></label>
    <label>pose_valid
      <select data-field="is_pose_valid">
        <option value="true" ${action.is_pose_valid ? 'selected' : ''}>true</option>
        <option value="false" ${!action.is_pose_valid ? 'selected' : ''}>false</option>
      </select>
    </label>
    <label>tcp <input data-speed="tcp_m_s" type="number" value="${speed.tcp_m_s ?? ''}"
      style="width:70px"/></label>
    <label>ori <input data-speed="ori_rad_s" type="number" value="${speed.ori_rad_s ?? ''}"
      style="width:70px"/></label>
    <label>joint <input data-speed="joint_rad_s" type="number"
      value="${speed.joint_rad_s ?? ''}" style="width:70px"/></label>
    <label>ext <input data-speed="ext_rad_s" type="number" value="${speed.ext_rad_s ?? ''}"
      style="width:70px"/></label>
    <select data-teach-point style="max-width:150px">${teachPointOptions()}</select>
  </div>
  <div class="cvrow row" style="margin-top:6px">
    ${Array.from({ length: 7 }).map((_, i) =>
      `<input data-cv-index="${i}" placeholder="cv${i}" type="number" step="any"
         value="${cv[i] ?? ''}" style="width:80px"/>`).join('')}
  </div>
  <div class="row" style="margin-top:6px">
    <button type="button" class="b-purple b-sm dupAction">复制</button>
    <button type="button" class="b-red b-sm delAction">删除</button>
    <button type="button" class="b-gray b-sm moveUpAction">↑</button>
    <button type="button" class="b-gray b-sm moveDownAction">↓</button>
  </div>`;
}

function renderBuilder() {
  const container = $('#groupsContainer');
  container.innerHTML = '';
  programState.actionGroup.forEach((group, gIdx) => {
    const card = document.createElement('div');
    card.className = 'group';
    card.dataset.groupIndex = gIdx;
    card.innerHTML = `
      <div class="group-head">
        <span style="font-weight:600">动作组 #${gIdx + 1}</span>
        <span style="font-size:12px;color:#6b7280">包含 ${group.length} 个动作</span>
        <span style="flex:1"></span>
        <button type="button" class="b-blue b-sm addAction">+ 动作</button>
        <button type="button" class="b-purple b-sm dupGroup">复制组</button>
        <button type="button" class="b-red b-sm delGroup"
          ${programState.actionGroup.length === 1 ? 'disabled' : ''}>删除组</button>
        <button type="button" class="b-gray b-sm moveUpGroup"
          ${gIdx === 0 ? 'disabled' : ''}>上移</button>
        <button type="button" class="b-gray b-sm moveDownGroup"
          ${gIdx === programState.actionGroup.length - 1 ? 'disabled' : ''}>下移</button>
      </div>
      <div class="group-body"></div>`;
    const body = card.querySelector('.group-body');
    group.forEach((action, aIdx) => {
      const row = document.createElement('div');
      row.className = 'action';
      row.dataset.actionIndex = aIdx;
      row.innerHTML = actionRowHTML(action);
      body.appendChild(row);
    });
    container.appendChild(card);
  });
  attachBuilderEvents();
  updatePreview();
}

function attachBuilderEvents() {
  const container = $('#groupsContainer');
  container.querySelectorAll('.addAction').forEach((btn) => {
    btn.onclick = (e) => {
      const g = +e.target.closest('[data-group-index]').dataset.groupIndex;
      programState.actionGroup[g].push(newEmptyAction());
      renderBuilder();
    };
  });
  container.querySelectorAll('.delGroup').forEach((btn) => {
    btn.onclick = (e) => {
      const g = +e.target.closest('[data-group-index]').dataset.groupIndex;
      if (programState.actionGroup.length > 1) {
        programState.actionGroup.splice(g, 1);
        renderBuilder();
      }
    };
  });
  container.querySelectorAll('.dupGroup').forEach((btn) => {
    btn.onclick = (e) => {
      const g = +e.target.closest('[data-group-index]').dataset.groupIndex;
      const c = JSON.parse(JSON.stringify(programState.actionGroup[g]));
      programState.actionGroup.splice(g + 1, 0, c);
      renderBuilder();
    };
  });
  container.querySelectorAll('.moveUpGroup').forEach((btn) => {
    btn.onclick = (e) => {
      const g = +e.target.closest('[data-group-index]').dataset.groupIndex;
      if (g > 0) { swap(programState.actionGroup, g, g - 1); renderBuilder(); }
    };
  });
  container.querySelectorAll('.moveDownGroup').forEach((btn) => {
    btn.onclick = (e) => {
      const g = +e.target.closest('[data-group-index]').dataset.groupIndex;
      if (g < programState.actionGroup.length - 1) {
        swap(programState.actionGroup, g, g + 1);
        renderBuilder();
      }
    };
  });
  container.querySelectorAll('.action[data-action-index]').forEach((row) => {
    const g = +row.closest('[data-group-index]').dataset.groupIndex;
    const a = +row.dataset.actionIndex;
    const action = programState.actionGroup[g][a];
    row.querySelectorAll('[data-field]').forEach((inp) => {
      inp.onchange = () => {
        const f = inp.dataset.field;
        let v = inp.value;
        if (['unitType', 'motionType'].includes(f)) v = parseInt(v || '0', 10);
        if (f === 'is_pose_valid') v = (v === 'true');
        action[f] = v;
        updatePreview();
      };
    });
    row.querySelectorAll('[data-speed]').forEach((inp) => {
      inp.onchange = () => {
        action.speed[inp.dataset.speed] = parseFloat(inp.value || '0');
        updatePreview();
      };
    });
    row.querySelectorAll('[data-cv-index]').forEach((inp) => {
      inp.onchange = () => {
        const v = parseFloat(inp.value);
        action.commandValue[+inp.dataset.cvIndex] = Number.isFinite(v) ? v : 0;
        updatePreview();
      };
    });
    const tpSel = row.querySelector('[data-teach-point]');
    if (tpSel) {
      tpSel.onchange = () => {
        if (tpSel.value === '') return;
        applyTeachPoint(action, +tpSel.value);
        renderBuilder();
      };
    }
    row.querySelector('.delAction').onclick = () => {
      programState.actionGroup[g].splice(a, 1);
      renderBuilder();
    };
    row.querySelector('.dupAction').onclick = () => {
      const c = JSON.parse(JSON.stringify(action));
      programState.actionGroup[g].splice(a + 1, 0, c);
      renderBuilder();
    };
    row.querySelector('.moveUpAction').onclick = () => {
      if (a > 0) { swap(programState.actionGroup[g], a, a - 1); renderBuilder(); }
    };
    row.querySelector('.moveDownAction').onclick = () => {
      if (a < programState.actionGroup[g].length - 1) {
        swap(programState.actionGroup[g], a, a + 1);
        renderBuilder();
      }
    };
  });
}

function updatePreview() {
  programState.name = $('#pbProgramName').value.trim();
  programState.cmd = $('#pbCmd').value.trim() || 'TESTAUTOPROGRAM';
  programState.actionGroup.forEach((group, g) =>
    group.forEach((a) => { a.currentActionGroup = g + 1; }));
  const errors = validateProgram(programState);
  const errBox = $('#builderErrors');
  if (errors.length) {
    errBox.classList.remove('hidden');
    errBox.textContent = errors.join('\n');
  } else {
    errBox.classList.add('hidden');
    errBox.textContent = '';
  }
  $('#jsonPreview').textContent = JSON.stringify(programState, null, 2);
}

async function sendProgram(payload, tag) {
  const r = await postCmd('/executeProgram', payload);
  const info = r.body && r.body.info ? ` (${r.body.info})` : '';
  pushLog(`${tag} → HTTP ${r.status}${info}`);
  if (!r.ok) toast(`程序下发失败: HTTP ${r.status}${info}`, true);
  return r.ok;
}

// ---- 循环执行（2026-10-13 服务端驱动：StateMaster 喂环+轮完成检测）----
const loop = { active: false, total: 0 };

function resetLoop() {
  loop.active = false;
  const btn = $('#loopControlBtn');
  btn.textContent = '开始循环';
  btn.classList.remove('b-red');
  btn.classList.add('b-orange');
  $('#loopProgress').textContent = '';
}

async function toggleLoop() {
  if (!loop.active) {
    if (!$('#loopExecuteEnabled').checked) { toast("请先勾选'循环执行'", true); return; }
    const infinite = $('#loopInfinite').checked;
    const count = infinite ? 0 : parseInt($('#loopCount').value, 10);
    if (!infinite && (!Number.isInteger(count) || count < 1)) {
      toast('循环次数至少为 1', true); return;
    }
    const errors = validateProgram(programState);
    if (errors.length) { toast('请先修复错误:\n' + errors.join('\n'), true); return; }
    // 先下发（一次），再开始循环；首轮启动由人工按「启动程序」（启动门控）
    const ok = await sendProgram(JSON.parse(JSON.stringify(programState)), '程序下发');
    if (!ok) return;
    const r = await postCmd('/loopProgram', { count });
    if (!r.ok) { toast('循环启动被拒（无程序？）', true); return; }
    loop.active = true; loop.total = count;
    const btn = $('#loopControlBtn');
    btn.textContent = '停止循环';
    btn.classList.remove('b-orange');
    btn.classList.add('b-red');
    pushLog(`循环已开始（服务端驱动，${infinite ? '无限' : count + ' 次'}）；首轮请按「启动程序」`);
  } else {
    await postCmd('/loopProgram', { count: -1 });
    resetLoop();
    pushLog('循环已停止');
  }
}

// ---- 后端程序存取 ----
async function loadPrograms() {
  const r = await api('/api/teach/programs');
  if (r.ok && r.body && Array.isArray(r.body.programs)) {
    App.teachPrograms = r.body.programs;
    const sel = $('#savedPrograms');
    sel.innerHTML = '<option value="">已保存程序…</option>' +
      App.teachPrograms.map((p, i) =>
        `<option value="${i}">${p.name || '程序' + (i + 1)}</option>`).join('');
  }
}

async function saveProgram() {
  const errors = validateProgram(programState);
  if (errors.length) { toast('请先修复错误:\n' + errors.join('\n'), true); return; }
  const name = programState.name || `程序${App.teachPrograms.length + 1}`;
  const copy = JSON.parse(JSON.stringify(programState));
  const existing = App.teachPrograms.findIndex((p) => p.name === name);
  const entry = { name, program: copy };
  if (existing >= 0) App.teachPrograms[existing] = entry;
  else App.teachPrograms.push(entry);
  const r = await postCmd('/api/teach/programs', { programs: App.teachPrograms });
  if (r.ok) { pushLog(`程序已保存: ${name}`); toast('程序已保存'); loadPrograms(); }
  else toast(`保存失败: ${r.body && r.body.info ? r.body.info : 'HTTP ' + r.status}`, true);
}

export function initProgram() {
  const root = document.getElementById('tab-program');
  root.innerHTML = `
    <div class="card"><h2>程序构建器</h2>
      <div class="row">
        <label>程序名称 <input id="pbProgramName" placeholder="name"/></label>
        <label>命令(cmd) <input id="pbCmd" value="TESTAUTOPROGRAM" readonly
          style="width:170px"/></label>
        <button class="b-blue b-sm" id="addGroupBtn">新增动作组</button>
        <select id="savedPrograms" style="max-width:160px">
          <option value="">已保存程序…</option></select>
        <button class="b-green b-sm" id="saveProgramBtn">保存程序</button>
      </div>
      <div id="groupsContainer" style="margin-top:10px"></div>
      <div id="builderErrors" class="hidden"
        style="color:#dc2626;font-size:13px;white-space:pre-line;margin-top:8px"></div>
      <h3 style="font-size:14px;margin:12px 0 4px">实时 JSON 预览</h3>
      <pre id="jsonPreview"
        style="background:#fff;border:1px solid #e5e7eb;border-radius:8px;padding:10px;
               font:12px/1.5 monospace;overflow:auto;max-height:280px;white-space:pre"></pre>
    </div>
    <div class="card"><h2>执行</h2>
      <div class="row">
        <button class="b-purple" id="sendProgramBtn">下发程序</button>
        <label><input type="checkbox" id="loopExecuteEnabled"/> 循环执行</label>
        <input id="loopCount" type="number" min="1" value="5" style="width:70px"
          placeholder="次数"/>
        <label><input type="checkbox" id="loopInfinite"/> 无限循环</label>
        <button class="b-orange" id="loopControlBtn">开始循环</button>
        <span id="loopProgress" style="font-size:13px;color:#2563eb"></span>
      </div>
      <div class="row" style="margin-top:8px">
        <button class="b-red" id="stopProgramBtn">停止程序</button>
      </div>
    </div>`;

  if (programState.actionGroup.length === 0) {
    programState.actionGroup.push([newEmptyAction()]);
  }
  $('#addGroupBtn').onclick = () => {
    programState.actionGroup.push([newEmptyAction()]);
    renderBuilder();
  };
  $('#pbProgramName').addEventListener('input', updatePreview);
  $('#sendProgramBtn').onclick = async () => {
    const errors = validateProgram(programState);
    if (errors.length) { toast('请先修复错误:\n' + errors.join('\n'), true); return; }
    await sendProgram(JSON.parse(JSON.stringify(programState)), 'executeProgram');
  };
  $('#loopControlBtn').onclick = toggleLoop;
  $('#stopProgramBtn').onclick = async () => {
    const r = await postCmd('/stopProgram', { StopProgram: true });
    pushLog(`stopProgram → HTTP ${r.status}`);
  };
  $('#saveProgramBtn').onclick = saveProgram;
  $('#savedPrograms').onchange = (e) => {
    const idx = e.target.value;
    if (idx === '') return;
    const entry = App.teachPrograms[+idx];
    if (!entry || !entry.program) return;
    Object.assign(programState, JSON.parse(JSON.stringify(entry.program)));
    $('#pbProgramName').value = programState.name || '';
    $('#pbCmd').value = programState.cmd || 'TESTAUTOPROGRAM';
    renderBuilder();
    pushLog(`已载入程序: ${entry.name}`);
  };
  loadPrograms();
  renderBuilder();
}

export function renderProgram() {
  // 程序页是编辑器，不随轮询重渲染（避免打断编辑）；只更新循环进度文本。
  const el = document.getElementById('loopProgress');
  if (!el || !loop.active) return;
  const q = App.state && App.state.queue;
  if (q && q.loopActive) {
    el.textContent = q.loopTotal > 0
      ? `循环中: ${q.loopRound}/${q.loopTotal}`
      : `循环中: 第 ${q.loopRound} 轮（无限）`;
  } else if (q && !q.loopActive && loop.active) {
    resetLoop();
    el.textContent = '循环已完成';
  }
}
