#ifndef __HMI_PAGE_HPP__
#define __HMI_PAGE_HPP__

namespace state_master_hmi {

// 单页 HMI：内嵌部署（零路径依赖），无外部 CDN（内网环境）。
// 接口：POST /stateMsg、/executeMovement、/executeProgram、/stopProgram、
//       /stateAndPositionFeedback（300ms 轮询）。
inline constexpr char kHmiPageHtml[] = R"HTML(<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>BBT HMI</title>
<style>
body{background:#f3f4f6;font-family:sans-serif;margin:0;padding:16px;color:#111}
h1{font-size:20px;text-align:center;color:#1d4ed8}
section{background:#fff;border-radius:10px;padding:12px 16px;margin:12px auto;max-width:960px;box-shadow:0 1px 4px #0002}
h2{font-size:15px;margin:4px 0 10px;color:#374151}
button{padding:6px 12px;margin:2px;border:0;border-radius:6px;background:#2563eb;color:#fff;cursor:pointer}
button:hover{background:#1d4ed8}
button.warn{background:#dc2626}
button.mode-on{background:#16a34a}
input,select,textarea{padding:5px;border:1px solid #d1d5db;border-radius:6px;margin:2px}
input[type=number]{width:80px}
textarea{width:100%;height:120px;font-family:monospace}
.bar{display:flex;gap:18px;flex-wrap:wrap;align-items:center}
.tag{padding:2px 10px;border-radius:10px;background:#e5e7eb;font-weight:600}
table{border-collapse:collapse;font-size:12px}
td,th{border:1px solid #e5e7eb;padding:2px 8px}
#log{font-size:12px;color:#6b21a8;white-space:pre-wrap;max-height:140px;overflow:auto}
</style>
</head>
<body>
<h1>BBT HMI 控制界面</h1>

<section>
  <h2>状态</h2>
  <div class="bar">
    <span>FSM：<span class="tag" id="fsmState">-</span></span>
    <span>模式：<span class="tag" id="runMode">-</span></span>
    <span id="clock"></span>
  </div>
  <div id="socTable" style="margin-top:8px"></div>
</section>

<section>
  <h2>模式切换</h2>
  <button id="btnManual" onclick="switchMode('msg_manual_mode')">MANUAL</button>
  <button id="btnAuto" onclick="switchMode('msg_auto_mode')">AUTO</button>
  <span style="font-size:12px;color:#6b7280">READY/MOTION 状态下切换会被拒绝（先下电）</span>
  <br><button onclick="sendState('msg_calib_zero')">捕获零位</button>
  <span style="font-size:12px;color:#6b7280">摆到 URDF 零位后点击；原始偏移打印在应用日志（ZeroPosition snippet），粘贴进 *_robot_conf.cfg 后重启生效</span>
</section>

<section>
  <h2>状态机命令</h2>
  <div id="fsmBtns"></div>
</section>

<section>
  <h2>手动动作（单步，到目标即停）</h2>
  <div>
    <select id="unitType"></select>
    <select id="motionType">
      <option value="1">1=关节</option>
      <option value="2">2=直线</option>
    </select>
    <select id="isPoseValid">
      <option value="false">关节值</option>
      <option value="true">位姿(x,y,z,qx,qy,qz,qw)</option>
    </select>
  </div>
  <div id="cvInputs"></div>
  <button onclick="sendMovement()">执行手动动作</button>
</section>

<section>
  <h2>程序执行（AUTO 模式）</h2>
  <textarea id="progJson">{"cmd":"TESTAUTOPROGRAM","actionGroup":[[{"unitType":0,"motionType":1,"name":"a","speed":{"tcp_m_s":0.1,"joint_rad_s":0.3},"is_pose_valid":false,"commandValue":[0,-0.4,0.4,0,0.4,0,0]}]]}</textarea>
  <br><button onclick="sendProgram()">下发程序</button>
  <button class="warn" onclick="stopProgram()">停止程序</button>
</section>

<section><h2>日志</h2><div id="log"></div></section>

<script>
const UNITS=["0 RIGHT_ARM","1 RIGHT_THUMB","2 RIGHT_FF","3 RIGHT_MF","4 RIGHT_RF","5 RIGHT_LF","6 LEFT_ARM","7 LEFT_THUMB","8 LEFT_FF","9 LEFT_MF","10 LEFT_RF","11 LEFT_LF","12 ROBOT_HEAD","13 ROBOT_BODY","14 AGV_FOOT"];
const FSM_CMDS=[["msg_initialize","初始化"],["msg_config","配置"],["msg_motor_on","上电"],["msg_motor_off","下电"],["msg_control_on","使能"],["msg_control_off","去使能"],["msg_reset_ok","重置"],["msg_estop","急停"]];
const $=id=>document.getElementById(id);
function log(s){$("log").textContent=(new Date().toLocaleTimeString())+" "+s+"\n"+$("log").textContent;}
window.onload=()=>{
  UNITS.forEach(u=>{$("unitType").innerHTML+=`<option value="${u.split(" ")[0]}">${u}</option>`;});
  FSM_CMDS.forEach(([m,label])=>{const b=document.createElement("button");b.textContent=label;if(m=="msg_estop")b.className="warn";b.onclick=()=>sendState(m);$("fsmBtns").appendChild(b);});
  let cv="";for(let i=0;i<7;i++)cv+=`<input type="number" step="0.01" id="cv${i}" placeholder="v${i}" value="0">`;
  $("cvInputs").innerHTML=cv;
  poll();setInterval(poll,300);
};
async function post(url,obj){
  const r=await fetch(url,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(obj)});
  const t=await r.text();return{status:r.status,text:t};
}
async function sendState(m){
  const r=await post("/stateMsg",{cmd:"REFSTATECHANGE",stateMSG:m});
  log(`状态命令 ${m} → ${r.status} ${r.text.replace(/\s+/g," ").slice(0,160)}`);
}
async function switchMode(m){await sendState(m);}
async function sendMovement(){
  const cv=[];for(let i=0;i<7;i++)cv.push(parseFloat($("cv"+i).value)||0);
  const r=await post("/executeMovement",{cmd:"MANUALPROGRAM",program:{
    unitType:parseInt($("unitType").value),motionType:parseInt($("motionType").value),
    is_pose_valid:$("isPoseValid").value==="true",commandValue:cv}});
  log(`手动动作 → ${r.status} ${r.text.replace(/\s+/g," ").slice(0,160)}`);
}
async function sendProgram(){
  let obj;try{obj=JSON.parse($("progJson").value);}catch(e){log("程序 JSON 解析失败: "+e.message);return;}
  const r=await post("/executeProgram",obj);
  log(`程序下发 → ${r.status} ${r.text.replace(/\s+/g," ").slice(0,160)}`);
}
async function stopProgram(){
  const r=await post("/stopProgram",{StopProgram:true});
  log(`停止程序 → ${r.status}`);
}
async function poll(){
  try{
    const r=await fetch("/stateAndPositionFeedback",{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"});
    const j=await r.json();
    $("fsmState").textContent=j.currentState||"-";
    $("runMode").textContent=j.runningMode||"-";
    $("btnManual").className=(j.runningMode==="manual")?"mode-on":"";
    $("btnAuto").className=(j.runningMode==="auto")?"mode-on":"";
    const socs=j.socFeedbacks||[];
    let h="<table><tr><th>SOC</th>"+Array.from({length:7},(_,i)=>`<th>J${i+1}</th>`).join("")+"</tr>";
    socs.forEach((s,i)=>{const ja=s.jointAngle||{};h+=`<tr><td>${UNITS[i]||i}</td>`+Array.from({length:7},(_,k)=>{const v=ja["J"+(k+1)];return `<td>${v===undefined?"":Number(v).toFixed(4)}</td>`;}).join("")+"</tr>";});
    $("socTable").innerHTML=h+"</table>";
  }catch(e){/* 服务端未就绪时静默 */}
}
</script>
</body>
</html>)HTML";

}  // namespace state_master_hmi
#endif
