// 主操作页：FSM 命令、模式切换、队列水位、消息日志。
import { App, postCmd, pushLog, toast } from '../app.js';

const FSM_BUTTONS = [
  ['msg_initialize', '初始化', 'b-blue'],
  ['msg_config', '配置', 'b-blue'],
  ['msg_motor_on', '上电', 'b-blue'],
  ['msg_motor_off', '下电', 'b-blue'],
  ['msg_control_on', '使能', 'b-green'],
  ['msg_control_off', '去使能', 'b-gray'],
  ['msg_reset_ok', '重置', 'b-purple'],
  ['msg_path_stop', '停止程序', 'b-orange'],
  // AUTO 启动门控（2026-10-13）：进 MOTION 不自动跑，按此开门
  ['msg_start_program', '启动程序', 'b-green'],
  ['msg_safe_stop', '回安全位', 'b-purple'],
  // 二期放开（2026-10-10）：Planning 重规划式断点恢复已交付
  ['msg_pause', '暂停', 'b-orange', false],
  ['msg_resume', '继续', 'b-green', false],
];

const MODE_BUTTONS = [
  ['msg_manual_mode', '手动模式'],
  ['msg_auto_mode', '自动模式'],
];

const QUEUE_LABELS = [
  ['manual', '手动队列', 8],
  ['program', '程序队列', 32],
  ['traj', '轨迹队列', 64],
  ['micro', '微步队列', 16],
];

async function sendStateMsg(msg) {
  const r = await postCmd('/stateMsg', { cmd: 'REFSTATECHANGE', stateMSG: msg });
  const info = r.body && r.body.info ? ` (${r.body.info})` : '';
  pushLog(`${msg} → HTTP ${r.status}${info}`);
  if (!r.ok) toast(`命令被拒: ${msg}${info}`, true);
}

export function initOperation() {
  const root = document.getElementById('tab-operation');
  root.innerHTML = `
    <div class="card"><h2>状态命令</h2><div class="row" id="fsmBtns"></div></div>
    <div class="card"><h2>模式切换</h2><div class="row" id="modeBtns"></div></div>
    <div class="card"><h2>队列水位</h2><div id="queueBars">—</div></div>
    <div class="card"><h2>消息日志</h2><div id="log"></div></div>`;
  const fsm = root.querySelector('#fsmBtns');
  for (const [msg, label, cls, disabled] of FSM_BUTTONS) {
    const b = document.createElement('button');
    b.className = cls;
    b.textContent = label;
    if (disabled) { b.disabled = true; b.title = '待 func FR-PS1 交付'; }
    b.addEventListener('click', () => sendStateMsg(msg));
    fsm.appendChild(b);
  }
  const mode = root.querySelector('#modeBtns');
  for (const [msg, label] of MODE_BUTTONS) {
    const b = document.createElement('button');
    b.className = 'b-blue';
    b.textContent = label;
    b.addEventListener('click', () => sendStateMsg(msg));
    mode.appendChild(b);
  }
}

export function renderOperation() {
  const q = App.state && App.state.queue;
  const el = document.getElementById('queueBars');
  if (el) {
    el.innerHTML = q
      ? QUEUE_LABELS.map(([key, label, cap]) => {
          const n = q[key];
          const pct = n >= 0 ? Math.min(100, (n / cap) * 100) : 0;
          const txt = n >= 0 ? `${n}/${cap}` : 'N/A';
          return `<div>${label} <span class="qwrap"><span class="qbar" ` +
                 `style="width:${pct}%"></span></span>${txt}</div>`;
        }).join('') +
        // AUTO 启动门控状态（2026-10-13）
        `<div>程序启动：${q.autoStarted ? '已启动' : '待启动'}</div>`
      : '—';
  }
  const log = document.getElementById('log');
  if (log) {
    log.textContent = App.log.join('\n');
    log.scrollTop = log.scrollHeight;
  }
}
