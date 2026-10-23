// 应用骨架：轮询循环 + 页签调度 + 公共工具。各页签模块挂在 App 上协作。
import { initOperation, renderOperation } from './tabs/operation.js';
import { initMonitor, renderMonitor } from './tabs/monitor.js';
import { initTeach, renderTeach, loadTeachPoints } from './tabs/teach.js';
import { initProgram, renderProgram } from './tabs/program.js';
import { initTwin, renderTwin } from './tabs/twin.js';

export const App = {
  describe: null,          // /api/robot/describe 结果
  state: null,             // 最新 /api/feedback 帧
  log: [],                 // 主操作页消息日志
  curve: { key: '', metric: 'p', buf: [] },  // 监控页曲线缓存（key="j.<logical>"）
  teachPoints: [],         // 示教点位（teach.js 维护，program.js 只读引用）
  teachPrograms: [],       // 已保存程序
};

const POLL_MS = 100;
const CURVE_MAX = 600;

export async function api(path, opts) {
  try {
    const res = await fetch(path, opts);
    const text = await res.text();
    let body = null;
    try { body = JSON.parse(text); } catch (_) { /* 非 JSON 响应（如 URDF） */ }
    return { ok: res.ok, status: res.status, body, bodyText: text };
  } catch (e) {
    return { ok: false, status: 0, body: null, bodyText: '', error: e.message };
  }
}

export function toast(msg, isErr = false) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.className = isErr ? 'err' : '';
  el.classList.remove('hidden');
  clearTimeout(toast._t);
  toast._t = setTimeout(() => el.classList.add('hidden'), 4000);
}

export function pushLog(line) {
  const ts = new Date().toLocaleTimeString();
  App.log.push(`${ts} ${line}`);
  if (App.log.length > 100) App.log.shift();
}

export function postCmd(path, data) {
  return api(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
  });
}

function setConn(ok) {
  document.getElementById('connState').innerHTML = ok
    ? '<span class="dot ok"></span>已连接'
    : '<span class="dot off"></span>未连接';
}

function renderTopbar() {
  const s = App.state;
  document.getElementById('fsmState').textContent = `状态: ${s ? s.state : '—'}`;
  document.getElementById('runMode').textContent = `模式: ${s ? s.mode : '—'}`;
  document.getElementById('staleFlag').classList.toggle('hidden', !(s && s.stale));
}

function renderActiveTab() {
  const active = document.querySelector('.tab.active').dataset.tab;
  if (active === 'operation') renderOperation(App);
  else if (active === 'monitor') renderMonitor(App);
  else if (active === 'teach') renderTeach(App);
  else if (active === 'program') renderProgram(App);
  else if (active === 'twin') renderTwin(App);
}

async function poll() {
  const r = await api('/api/feedback');
  if (r.ok) { App.state = r.body; setConn(true); } else { setConn(false); }
  // 曲线环形缓存（仅监控页选中轴）
  const c = App.curve;
  if (c.key && App.state && !App.state.stale) {
    const logical = parseInt(c.key.split('.')[1], 10);
    const j = App.state.joints[logical];
    if (j) {
      c.buf.push(j[c.metric]);
      if (c.buf.length > CURVE_MAX) c.buf.shift();
    }
  }
  renderTopbar();
  renderActiveTab();
}

function initTabs() {
  document.querySelectorAll('.tab').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.tab').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      document.querySelectorAll('.tabpage').forEach((p) => p.classList.add('hidden'));
      document.getElementById(`tab-${btn.dataset.tab}`).classList.remove('hidden');
      location.hash = btn.dataset.tab;   // #tab 直达，便于分享与自动化验证
      renderActiveTab();
    });
  });
  // URL hash 直达页签（如 #twin）
  const hash = location.hash.slice(1);
  const target = hash && document.querySelector(`.tab[data-tab="${hash}"]`);
  if (target) target.click();
}

function initEstop() {
  document.getElementById('estopBtn').addEventListener('click', async () => {
    if (!window.confirm('确认急停？')) return;
    const r = await postCmd('/stateMsg', { cmd: 'REFSTATECHANGE', stateMSG: 'msg_estop' });
    pushLog(`estop → HTTP ${r.status}`);
    if (!r.ok) toast(`急停命令失败: HTTP ${r.status}`, true);
  });
}

async function boot() {
  const d = await api('/api/robot/describe');
  if (d.ok) {
    App.describe = d.body;
    document.title = `机器人 HMI (${d.body.flavor})`;
  } else {
    toast('获取机器人描述失败，页面功能受限', true);
  }
  initEstop();
  initOperation(App);
  initMonitor(App);
  initTeach(App);
  initProgram(App);
  initTwin(App);   // 只建外壳；three 场景推迟到页签首次可见（renderTwin 触发）
  loadTeachPoints();   // 异步加载点位，失败只 toast 不阻塞
  initTabs();      // 最后执行：hash 直达点击可能触发页签 render，需各页 init 完成
  setInterval(poll, POLL_MS);
  poll();
}

boot();
