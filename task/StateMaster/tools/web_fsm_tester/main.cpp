/** @file main.cpp
 *  @brief Web 手动状态机测试台（sudo 运行，监听 8090）
 */
#include <signal.h>
#include <unistd.h>

#include <cstdio>
#include <memory>
#include <string>

#include "httplib.h"
#include "json.h"

#include "api_utils.h"
#include "app_control.h"
#include "hmi_client.h"
#include "sequence_runner.h"
#include "shm_watcher.h"

using namespace web_fsm_tester;

namespace {

pid_t g_launcher_pid = -1;
ShmStatusWatcher g_watcher;
HmiClient g_hmi(8080);
SequenceRunner g_seq;
httplib::Server g_svr;

constexpr int kWebPort = 8090;

bool AppRunning() {
    return g_launcher_pid > 0 && process_exists("taskframe_launcher");
}

void SignalHandler(int) { g_svr.stop(); }

}  // namespace

int main() {
    if (getuid() != 0) {
        fprintf(stderr, "[warn] 非 root 运行：EtherCAT 将无法就绪（需 sudo）\n");
    }
    if (process_exists("taskframe_launcher")) {
        fprintf(stderr, "[warn] 检测到已在运行的 taskframe_launcher，/api/start 将先清理它\n");
    }

    g_svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        static const char kPage[] = R"HTML(<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>状态机手动测试台</title>
<style>
body{font-family:sans-serif;background:#f3f4f6;margin:0;padding:24px;color:#111827}
.card{max-width:860px;margin:0 auto;background:#fff;border-radius:12px;box-shadow:0 2px 8px rgba(0,0,0,.08);padding:24px}
h1{text-align:center;color:#2563eb;font-size:22px;margin:0 0 20px}
h2{font-size:16px;margin:18px 0 8px;color:#374151}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
button{padding:8px 14px;border:0;border-radius:8px;color:#fff;cursor:pointer;font-size:14px}
button:disabled{opacity:.35;cursor:not-allowed}
.b-blue{background:#3b82f6}.b-gray{background:#6b7280}.b-green{background:#16a34a}
.b-orange{background:#f59e0b}.b-red{background:#ef4444}.b-purple{background:#8b5cf6}
table{width:100%;border-collapse:collapse}
td,th{padding:6px 10px;border-bottom:1px solid #e5e7eb;text-align:left;font-size:14px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
.ok{background:#22c55e}.warn{background:#f59e0b}.err{background:#ef4444}.off{background:#9ca3af}
#log{background:#111827;color:#d1d5db;border-radius:8px;padding:10px;height:160px;overflow-y:auto;font:12px/1.5 monospace;white-space:pre-wrap}
#seqbar{font-size:14px;color:#374151}
</style>
</head>
<body>
<div class="card">
<h1>状态机手动测试台</h1>

<h2>应用控制</h2>
<div class="row">
  <button class="b-green" onclick="startApp()">启动应用</button>
  <button class="b-red" onclick="stopApp()">停止应用</button>
  <span id="runState"><span class="dot off"></span>未运行</span>
</div>

<h2>进程状态（每秒刷新）</h2>
<table id="taskTable"><tr><th>进程</th><th>状态</th><th>指示</th></tr></table>

<h2>位置反馈（每秒刷新）<span id="fbState" style="font-size:13px;color:#6b7280"></span></h2>
<table id="fbTable"><tr><td style="color:#9ca3af">反馈不可用</td></tr></table>

<h2>手动命令</h2>
<div class="row">
  <button class="b-blue" data-msg="msg_initialize" onclick="sendCmd(this)">初始化</button>
  <button class="b-blue" data-msg="msg_config" onclick="sendCmd(this)">配置</button>
  <button class="b-blue" data-msg="msg_motor_on" onclick="sendCmd(this)">上电</button>
  <button class="b-blue" data-msg="msg_motor_off" onclick="sendCmd(this)">下电</button>
  <button class="b-blue" data-msg="msg_control_on" onclick="sendCmd(this)">使能</button>
  <button class="b-blue" data-msg="msg_control_off" onclick="sendCmd(this)">去使能</button>
  <button class="b-orange" data-msg="msg_error" onclick="sendCmd(this)">错误</button>
  <button class="b-purple" data-msg="msg_reset_ok" onclick="sendCmd(this)">重置</button>
  <button class="b-red" data-msg="msg_estop" onclick="sendCmd(this)">急停</button>
</div>

<h2>一键完整序列</h2>
<div class="row">
  <button class="b-green" onclick="seqStart()">▶ 开始序列</button>
  <button class="b-gray" onclick="seqStop()">■ 中止</button>
  <span id="seqbar">空闲</span>
</div>

<h2>日志</h2>
<div id="log"></div>
</div>
<script>
const ALLOWED={
  0:["msg_initialize","msg_error","msg_estop"],
  1:["msg_config","msg_error","msg_estop"],
  2:["msg_motor_on","msg_error","msg_estop"],
  3:["msg_control_on","msg_motor_off","msg_error","msg_estop"],
  4:["msg_control_off","msg_motor_off","msg_error","msg_estop"],
  5:["msg_reset_ok"],
  127:["msg_reset_ok"]
};
const logs=[];
let lastSeqFinished=false;
function log(m){
  const t=new Date().toLocaleTimeString();
  logs.unshift(`[${t}] ${m}`);
  if(logs.length>200)logs.pop();
  document.getElementById('log').textContent=logs.join('\n');
}
async function post(url,body){
  const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});
  return r.json();
}
async function startApp(){log('启动应用（约 5-20s）...');const r=await post('/api/start');log(r.ok?'应用已启动':'启动失败: '+r.detail);}
async function stopApp(){log('停止应用...');const r=await post('/api/stop');log(r.ok?'已停止':'停止失败: '+(r.detail||''));}
async function sendCmd(btn){const msg=btn.dataset.msg;log('下发 '+msg);const r=await post('/api/command',{msg});log(r.ok?('已入队 '+msg):('失败: '+r.detail));}
async function seqStart(){log('启动一键序列');const r=await post('/api/sequence/start');if(!r.ok)log('序列启动失败: '+r.detail);}
async function seqStop(){await post('/api/sequence/stop');log('已请求中止序列');}
function render(s){
  document.getElementById('runState').innerHTML=s.running
    ?'<span class="dot ok"></span>运行中':'<span class="dot off"></span>未运行';
  const tbl=document.getElementById('taskTable');
  tbl.innerHTML='<tr><th>进程</th><th>状态</th><th>指示</th></tr>';
  let allSame=true,first=null;
  for(const t of s.tasks){if(first===null)first=t.status;else if(t.status!==first)allSame=false;}
  for(const t of s.tasks){
    const cls=t.status===127?'err':(allSame?'ok':'warn');
    tbl.innerHTML+=`<tr><td>${t.name}</td><td>${t.statusName}</td><td><span class="dot ${cls}"></span></td></tr>`;
  }
  const master=s.tasks.length?s.tasks[0].status:-1;
  const allowed=s.running?(ALLOWED[master]||[]):[];
  document.querySelectorAll('button[data-msg]').forEach(b=>{
    b.disabled=s.seq.active||!allowed.includes(b.dataset.msg);
  });
  const q=s.seq,bar=document.getElementById('seqbar');
  if(q.active)bar.textContent=`第 ${q.step}/${q.stepCount} 步 ${q.label} — 执行中...`;
  else if(q.finished)bar.textContent=q.success?'序列完成 ✔':('序列失败: '+q.error);
  else bar.textContent='空闲';
  if(q.finished&&!lastSeqFinished){log(q.success?'一键序列完成 ✔':'一键序列失败: '+q.error);}
  lastSeqFinished=q.finished;
}
let lastFb=null;
function renderFeedback(d){
  const tbl=document.getElementById('fbTable');
  const st=document.getElementById('fbState');
  if(!d||!Array.isArray(d.socFeedbacks)){
    if(lastFb){ // 保留最后一次成功数据，仅标记异常
      st.textContent=' 当前状态: '+(lastFb.currentState||'?')+'（更新失败，显示旧值）';
    } else {
      tbl.innerHTML='<tr><td style="color:#9ca3af">反馈不可用</td></tr>';
      st.textContent='';
    }
    return;
  }
  lastFb=d;
  st.textContent=' 当前状态: '+(d.currentState||'?');
  const cols=new Set();
  for(const u of d.socFeedbacks){const ja=u.jointAngle||{};for(const k in ja)cols.add(k);}
  const keys=[...cols].sort((a,b)=>parseInt(a.slice(1))-parseInt(b.slice(1)));
  let html='<tr><th>单元</th>'+keys.map(k=>`<th>${k}</th>`).join('')+'</tr>';
  d.socFeedbacks.forEach((u,i)=>{
    const ja=u.jointAngle||{};
    html+=`<tr><td>单元 #${i}</td>`+
      keys.map(k=>`<td>${ja[k]!==undefined?Number(ja[k]).toFixed(4):'—'}</td>`).join('')+
      '</tr>';
  });
  tbl.innerHTML=html;
}
async function pollFb(){
  try{
    const r=await fetch('/api/feedback');
    const d=await r.json();
    renderFeedback(d&&d.ok===false?null:d);
  }catch(e){renderFeedback(null);}
}
async function poll(){try{const r=await fetch('/api/status');render(await r.json());}catch(e){}pollFb();}
setInterval(poll,1000);poll();
</script>
</body>
</html>
)HTML";
        res.set_content(kPage, "text/html");
    });

    g_svr.Get("/api/status", [](const httplib::Request&, httplib::Response& res) {
        std::vector<TaskStatusInfo> tasks;
        if (g_watcher.Attach()) {
            tasks = g_watcher.ReadAll();
        } else {
            for (const char* name : kTaskNames) tasks.push_back({name, -1});
        }
        res.set_content(BuildStatusJson(AppRunning(), tasks, g_seq.View()),
                        "application/json");
    });

    g_svr.Get("/api/feedback", [](const httplib::Request&, httplib::Response& res) {
        if (!AppRunning()) {
            res.set_content(R"({"ok":false,"detail":"app not running"})",
                            "application/json");
            return;
        }
        std::string body, err;
        if (g_hmi.GetPositionFeedback(&body, &err)) {
            res.set_content(body, "application/json");  // 透明中继
        } else {
            Json::Value root;
            root["ok"] = false;
            root["detail"] = err;
            Json::StreamWriterBuilder wb;
            res.set_content(Json::writeString(wb, root), "application/json");
        }
    });

    g_svr.Post("/api/command", [](const httplib::Request& req, httplib::Response& res) {
        Json::Value body;
        Json::CharReaderBuilder rb;
        std::string errs;
        std::unique_ptr<Json::CharReader> reader(rb.newCharReader());
        bool parsed = reader->parse(req.body.data(), req.body.data() + req.body.size(),
                                    &body, &errs);
        std::string msg = parsed ? body.get("msg", "").asString() : "";
        Json::Value root;
        if (!IsValidStateMsg(msg)) {
            root["ok"] = false;
            root["detail"] = "invalid msg: " + msg;
        } else if (g_seq.IsActive()) {
            root["ok"] = false;
            root["detail"] = "sequence running";
        } else if (!AppRunning()) {
            root["ok"] = false;
            root["detail"] = "app not running";
        } else {
            std::string err;
            if (g_hmi.SendFsmCommand(msg, &err)) {
                root["ok"] = true;
                root["detail"] = "queued";
            } else {
                root["ok"] = false;
                root["detail"] = err;
            }
        }
        Json::StreamWriterBuilder wb;
        res.set_content(Json::writeString(wb, root), "application/json");
    });

    g_svr.Post("/api/start", [](const httplib::Request&, httplib::Response& res) {
        Json::Value root;
        if (AppRunning()) {
            root["ok"] = false;
            root["detail"] = "already running";
        } else {
            g_launcher_pid = LauncherStart();
            if (g_launcher_pid <= 0) {
                root["ok"] = false;
                root["detail"] = "fork failed";
            } else {
                // LauncherStart 重建了 shm：先 Detach 掉可能持有的旧映射，
                // 否则 StatusTableReady/状态读取会看到旧 shm 的冻结数据
                g_watcher.Detach();
                // 等 4 进程全部注册进状态表（~20s 足够覆盖启动路径）
                bool ready = false;
                for (int waited = 0; waited < 20000 && !ready; waited += 50) {
                    ready = g_watcher.StatusTableReady();
                    if (!ready) usleep(50000);
                }
                if (ready) {
                    root["ok"] = true;
                    root["detail"] = "started";
                } else {
                    root["ok"] = false;
                    root["detail"] = "workers not ready within 20s: " + g_watcher.Snapshot();
                }
            }
        }
        Json::StreamWriterBuilder wb;
        res.set_content(Json::writeString(wb, root), "application/json");
    });

    g_svr.Post("/api/stop", [](const httplib::Request&, httplib::Response& res) {
        g_seq.Shutdown();   // 停应用前先停序列线程
        LauncherStop(g_launcher_pid);
        g_launcher_pid = -1;
        g_watcher.Detach();
        Json::Value root;
        root["ok"] = true;
        Json::StreamWriterBuilder wb;
        res.set_content(Json::writeString(wb, root), "application/json");
    });

    g_svr.Post("/api/sequence/start", [](const httplib::Request&, httplib::Response& res) {
        Json::Value root;
        if (!AppRunning()) {
            root["ok"] = false;
            root["detail"] = "app not running";
        } else if (!g_seq.Start(&g_watcher, &g_hmi)) {
            root["ok"] = false;
            root["detail"] = "sequence already running";
        } else {
            root["ok"] = true;
        }
        Json::StreamWriterBuilder wb;
        res.set_content(Json::writeString(wb, root), "application/json");
    });

    g_svr.Post("/api/sequence/stop", [](const httplib::Request&, httplib::Response& res) {
        g_seq.RequestStop();
        Json::Value root;
        root["ok"] = true;
        Json::StreamWriterBuilder wb;
        res.set_content(Json::writeString(wb, root), "application/json");
    });
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    printf("web_fsm_tester listening on 0.0.0.0:%d\n", kWebPort);
    g_svr.listen("0.0.0.0", kWebPort);

    g_seq.Shutdown();
    if (AppRunning()) LauncherStop(g_launcher_pid);
    return 0;
}
