# 追加批次（2026-10-12：ppwodyn 程序速度逐组限幅 PS1 + joint-run 计时修复链）

- `libppwodyn_humanoid.a` / `libppwodyn_dualarm.a`
- **无 ABI 破坏**：`RunAll` 1 参符号原样保留；新增 2 参重载
  `RunAll(waypoints, const KinematicLimits* limits_override)`。
- override 语义：nullptr / 字段<=0 / NaN / ==NSDMI 标记值（0.25/1.05，即
  `KinematicLimits{}` 值初始化默认内容）→ 该字段用填充后默认；其余正值 →
  min(override, 填充后默认)（只收不放）；仅 v_max/v_joint_max 生效。
- **⚠️ 有意的行为变化 1（裁定）**：规划侧揭开手动模式 v_max=0.25 →
  min(10, linear_max_speed 若已填充)，默认路径同。无 override 的 LINE 类
  程序规划变快。手动模式 0.25 限速由 task 侧实现（见关联需求文档）。
- **⚠️ 有意的行为变化 2（连带修复，用户裁定）**：joint-run 计时恢复物理
  量级——旧版因 profiler dq_dL 折算误除 seg_len 虚增细分数 n 倍
  （时长 ∝ 1/v_joint_max²；实测 2rad@1rad/s 旧 172s → 新 2.12s）。
  修复链：profiler 按 action 跨度折算 + Step4 LINE 分支同型修正 +
  纯 joint run 跳过 IK 恢复与笛卡尔导数清零 + TOPP 补向心速度钳
  （含 1e-9 微降额消边界浮点误判）。混合序列 phase2 不再因该链假不可行
  而跳过。
- shm 布局不变；pathplanning 库同型嫌疑另立 follow-up（spec §9.2）。
- 需求：`docs/2026-10-12-func-program-speed-override-requirement.md`；
  spec（含 §9 执行期实证链）：`docs/superpowers/specs/2026-10-12-program-speed-override-design.md`；
  task 指南：`docs/2026-10-12-func-program-speed-override-delivery.md`；
  task 手动限速需求：`docs/2026-10-12-task-manual-mode-speed-limit-requirement.md`。
- 回归：双 flavor 全绿（humanoid 13/13、dualarm 17/17；新
  test_speed_override 5 用例：减半 ratio=1.9134≈加倍、clamp 收住、
  nullptr/标记值/零/NaN/负值零回退、LINE v_max 独立、1参≡2参nullptr）。

# 追加批次（2026-10-10：servo PathStop 断点续跑 FR-PS1）

- `libservo_humanoid.a` / `libservo_dualarm.a`
- **ABI 破坏（需重链）**：`PathStop::GenerateBraking` 加默认尾参
  `double* l_stop = nullptr`（mangled name 变化；源码兼容，调用方零改动）。
- 纯新增：`PathStop::GenerateResume`——从零速停点三次加速接回原时间律，
  覆盖 [l_stop, 1]；`l_stop>=1` 空剖面 true；接合不可行时 false + 步数据
  有效（镜像 GenerateBraking 越限语义）。
- shm 布局不变；SplineInterpolation 不变。
- 需求：`docs/2026-10-10-func-pathstop-resume-requirement.md`；
  spec：`docs/superpowers/specs/2026-10-10-pathstop-resume-design.md`；
  task 指南：`docs/2026-10-10-func-pathstop-resume-delivery.md`。
- 回归：双 flavor ctest 全绿（R1 出参/参数校验/空剖面/速度接合/不可行/
  E2E 连续性含 matplotlib 图/1kHz×1s 热路径）。

# 追加批次（2026-10-10：运动学错误处理整改 P0–P3 全量实施）

设计报告：`docs/2026-10-10-kinematic-error-handling-design-report.md`
（含附录 A 完整码表）；计划：`docs/superpowers/plans/2026-10-10-kinematic-error-handling-remediation.md`

⚠️ **本批次含多重 ABI/行为破坏，task 侧必须重链 + 适配**：

1. **统一码约定封死**：`1`=OK、正=warning、负=error、**`0`=非法/未初始化
   哨兵**（任何求解路径不得返 0）。手指/拇指 `0=成功` 符号反转为 `1=OK`；
   task 侧凡 `== 0`/`!rc` 判手指/拇指 Calc* 处改 `> 0`。
2. **存根显式化**：CBaseTarget 四虚方法存根、手指 `CalcJointWithConfig`/
   `CalcVelocityJacobian` 等「静默假成功」全部改返 `-203`
   （ROBOT_TARGET_ERROR_MODEL_NOT_IMPL）。
3. **bool→int**（bool 不能承载 warning 码，false 撞 0 哨兵）：
   `CBaseTarget::CalcVelocityJacobian`（虚接口）、`CArmTarget::
   CalcVelocityJacobian(/Nom)`（double[6][6]）、`CArmTarget::CalcJointSing`、
   `CTarget::CalcVelocityJacobian`。
4. **kNieRobot 落空 6 处** `return 0` → `-203`（修订 10-08 裁定，
   §3.8 冲突消解）。
5. **新码**：不可达子码 `-111/-112/-113/-114*/-115`（-114 服务 IRB 第 4 级，
   肘式遗产为死码）、warning `20`（四元数未归一化，遗产恢复）。
   ~~耦合/限位段 -401~-404~~ 已取消（用户裁定：当前机器人构型不需要
   耦合检查）。**IRB 撞号修复**：`IRB -101/-102/-123/-200`
   与统一表语义撞号（-102 实为「θ2 无解」被误读为「姿态不可达」）——
   现统一映射到 -11x/-203；**task 侧日志/监控按码值硬匹配处需排查**。
6. **详情通道**：`KinematicErrorInfo`（POD，可选尾参，nullptr 零开销）——
   IK 入口：`CArmTarget::CalcJointCnf/CalcJointNoCnf`、`CKinGeneRbt` 四公开
   方法、`CNieTarget::CalcJoint`。默认参数改签名属 ABI 破坏。
7. **NIE 奇异 warning**：CNieTarget::CalcJoint 在 J5≈0（|sin|<1e-3）返回
   `ROBOT_TARGET_WRIST_SINGULARITY(11)`（结果照常写回）。
8. **日志门控**：`TargetData.elog_enable` 复活——工况日志（IK 失败等）
   按 target 数据门控（默认关）；契约违反（未 Attach/初始化）仍强制。
   kinplinkrbt 遗产 19 处 ELOG 注释点恢复为门控日志。
9. **内部码族退役**：`ELBOW_TARGET_*`/`IRB_TARGET_*` 宏删除，求解器直出
   统一码；`ElbowToRobotCode`/`IrbToRobotCode` 翻译层删除。
   附带修复：`CKinGeneRbt::CalcJointNoConfigNominal` 此前直泄内部码
   （-1001s 未翻译），已纳入统一边界。
10. **码表下沉**：`ROBOT_TARGET_*` 宏从 armtarget.hpp 移到 basetarget.hpp
    （基类/手指/拇指可见性需要）；引用方无需改（armtarget.hpp 仍传递包含）。
11. **取消项**（P1⑤）：耦合/检查点检查 API（check_joint_coupling 等）
    不恢复——用户裁定当前机器人构型不需要耦合检查；-4xx 码段不启用。

回归证据：kinematic 双 flavor 31+26 全绿；下游 kinplanning 9+8、
action 9+9、pathplanning 27+22、ppwodyn 16+12 全绿（全量删二进制重链）。
性能：详情填充额外开销实测为负（噪声级），上限断言钉住。
task 侧升级指南：`docs/2026-10-10-func-error-handling-task-handoff.md`。

---

# 追加批次（2026-10-09：confbase 57 宽契约适应 + NIE 米制根治 + 补丁清理）

驱动：base 57 宽契约修复交付
（`docs/2026-10-09-confbase-dualarm-shm-layout-contract-response-base.md`，
`libconfbase_<flavor>.a` 09:14）。
设计/计划：`docs/superpowers/specs/2026-10-09-func-confbase-57wide-adaptation-design.md`、
`docs/superpowers/plans/2026-10-09-func-confbase-57wide-adaptation.md`

变更摘要：
1. **servo 转换桥删除**（编译强制）：base 将 `DualArmGlobalJointMap/Params`
   别名化为通用 57 宽类型后，`crobot_transformer.cpp` 的
   `ToGenericMap/ToGenericParams` 双 flavor 四函数签名两两相同
   （dualarm redefinition 编译错误）。别名化后桥即恒等——全部删除，
   `JointToMotor`/`GetUnitJointContext` 直传 store 输出；`ToGenericUnit`
   reinterpret 桥保留（UnitConfig 仍独立 struct）。**行为零变化**
   （桥拷贝的正是同值前缀）。双 flavor 45/45 绿。
2. **nie_arm_conf 米制化**：缺省几何/MDH 常量毫米→米（NIE_L0..L3/LT ÷1000，
   NIE_A/NIE_D 随动），消除「空参 fallback=毫米 / confbase 注入=米」混单位。
   ⚠️ **行为变化**：仅限空参 fallback 路径（手工填槽/旧 bin）输出从毫米变米；
   confbase 注入路径本就米制不变。新增 `NieTargetUnits.DefaultsAreMeters`
   锁定。task 侧若有按毫米解释 NIE fallback 输出的代码需同步。
3. **测试补丁清理**：`PatchRobotTopology` 收窄为仅补 `chain_tip_link`
   （unit_types/链名 builder 已正确填充，收窄版先校验再补）；`PatchJointDrive
   SpeedLimits` **保留**——更正研判：速度限位填充源是 URDF `<limit velocity>`，
   合成测试 URDF（dynamics/2arm_urdf_v1）velocity=0 仍需补丁
   （`EnsureElbowBackendConfigAttached` 等调用链不变）。`PatchDriveChain
   TransmissionRatios`/`PatchKinematicMdhParams`/`PatchArmModelNames` 保留。
4. **base 新需求**：`docs/2026-10-09-confbase-drivechain-tip-link-requirement.md`
   ——`transmission_ratio` 恒 NaN（双 flavor）+ `chain_tip_link` 空。

回归证据（全部对 09:14 confbase）：servo 45+45、dyn_humanoid 15/15、
pathplanning_humanoid 12/12、dualarm kinematic 30 / kinplanning 9 /
pathplanning 27 / ppwodyn 16 / action 9、humanoid 25/8/22/12/9 全绿；
nm 守卫 humanoid 无 CNie* 符号。

⚠️ task 侧行动项：重链 servo + kinematic（及下游）；NIE fallback 米制化
若涉生产路径请确认；real-machine 验收按 base 回复 §6 执行。

---

# 追加批次（2026-10-08：dualarm kinematic_model NIE 正名 + CArmTarget 委派 + 红灯闭环）

设计/计划：`docs/superpowers/specs/2026-10-08-dualarm-kinematic-model-nie-design.md`、
`docs/superpowers/plans/2026-10-08-dualarm-kinematic-model-nie.md`

变更摘要：
1. **cfg 改名**（`cfg/dualarm_robot_conf.cfg`）：`kinematic_model` 双臂
   `"CKinGeneRbt"`→`"CNieTarget"`、新增 `robot_body: "CNieWaist"`。
   ⚠️ task 侧生产 cfg 需同步此三条目；旧 bin（无新名）由「未识别名兜底标
   kNieRobot」补丁兼容，行为不劣化。
2. **注册表**：`KinematicModelClass` 新增 `kArmNie`（"CNieTarget"）/
   `kNieWaist`（"CNieWaist"），flavor 中立。
3. **标注正路**：`InitializeFromRobot` 对 kArmNie 显式标 `kNieRobot`；
   kNieWaist 不写 robot_type（腰非臂概念）。
4. **CArmTarget NIE 委派**：`robot_type=kNieRobot` 时
   `CalcPose/CalcPoseNom/CalcJointCnf/CalcJointNoCnf/CalcJointNoCnfNom/
   CalcJointCnfNom/CalcVelocityJacobian/CalcVelocityJacobianNom` 八方法委派
   栈上 `CNieTarget`（`#if DUAL_ARM` 门控，humanoid 库无 CNie 符号）。
   腰角经新视图成员 `waist_data_`（**非 POD，shm 布局不变**）从
   robot_body 槽（1-DOF，`joints[0]`）读取；`CNieGroup::Attach` 自动注入，
   未注入时腰角=0 + Log_W 兜底。
5. **resolver 路由修复**：dualarm 下 `KinematicsResolver` 此前因
   "CKinGeneRbt" 不含 "nie" 子串**错选 HumanoidKinematicsBackend**（探针
   实证）；cfg 改名后正确选中 NieKinematicsBackend，回归测试
   `test_resolver_nie_dispatch`（含反向验证）钉住。
6. **pathplanning_humanoid 2 红灯闭环**（同日上午）：根因为 09-30 cfg
   把 `ratio.robot_body` WJ1 从 120 改为 1（WJ1 力矩上限 0.07Nm < 重力矩
   6.19Nm → TOPP 回退），非 confbase 代码。测试侧 patch
   （`test_config_patch.hpp`）扩展 WJ1=120，库代码零改动。
   ⚠️ task 侧待确认 WJ1 真实减速比回填生产 cfg。

⚠️ **ABI 破坏**（task 侧全链重链）：
- `CArmTarget` 新增 `waist_data_` 指针成员 → `CArmTarget/CTarget/
  CTargetGroup` 视图类尺寸变化（sizeof(CTarget) 2504→2512）。shm 布局不变。
- **行为变化**：dualarm 双臂 robot_type kElbowRobot→kNieRobot；单槽
  CArmTarget::Calc* 改走 NIE 求解器（数值与肘式不同——NIE 为正确模型）。
- **测试适配先例**：语义绑定肘式模型的既有测试（pathplanning
  ik_recovery/ik_seed、ppwodyn dualarm mixed E2E）已改用
  `EnsureElbowBackendConfigAttached()`（bin 内模型名 patch 回 CKinGeneRbt）。
  task 侧若有同类「按肘式模型调制路点」的测试/工具，需要同模式处理。

⚠️ **链接陷阱（本次实证新增）**：`link_libraries(<name>)` 纯名字链接
**不产生文件级 make 依赖**——下游测试二进制不会因上游 `.a` 更新而自动
重链，陈旧库混链表现为 stack smashing/segfault。凡上游库重建，下游必须
删测试二进制强制重链（本批次 pathplanning/ppwodyn 各踩一次）。

回归证据：dualarm kinematic 30 / kinplanning 9 / pathplanning 27 /
ppwodyn 16 / action 9；humanoid 25/8/22/12/9；pathplanning_humanoid 12/12；
全链删二进制重链后全绿。委派 FK 批均值 16.1µs/次（1000 次）。
nm 守卫：humanoid 库无 CNie* 符号。

---

# 追加批次（2026-10-01：TargetGroup 继承体系统一 + 紧凑槽位 + 错误码统一）

需求链：`docs/2026-09-30-func-initialize-from-robot-dualarm-slot-mapping.md`
（task 联署，dualarm 手动模式实机阻塞）
设计/计划：`docs/superpowers/specs/2026-09-30-target-group-inheritance-unification-design.md`、
`docs/superpowers/plans/2026-09-30-target-group-inheritance-unification.md`

变更摘要：
1. **dualarm 紧凑槽位**（task 需求本体）：`CTargetGroup::InitializeFromRobot`
   槽位映射 dualarm 下改为紧凑枚举 RIGHT_ARM=0/LEFT_ARM=1/ROBOT_BODY=2
   （`kinematic::ToLegacySocUnitType` 双 flavor 分支；新增别名 `ToSlotIndex`）。
   humanoid flavor 布局零变化（0/6/12/13/14）。新增 flavor 语义常量
   `kSocRightArm/kSocLeftArm/kSocBody`（legacy_robot_types.hpp），
   CTarget 路由 switch、CRM 门控、ToConfigSocType 三处拷贝等全部改用。
2. **继承体系**：`CTargetGroup` → `CNieGroup`（dualarm：紧凑槽位常量 +
   NIE 求解器/腰 view/CalcDual*）与 `CHumanoidGroup`（humanoid 最小骨架：
   legacy 槽位常量 + 类型化访问器）。编译期隐藏，无虚函数。
   kinplanning `AttachNieGroup`→`AttachGroup(CTargetGroup*)`（旧名别名
   仅 dualarm 保留一轮）、`CompositeRobotModel::Init` 只收 `CTargetGroup`。
3. **CNieGroup::Attach 不再硬填槽位**：槽位数据必须由 InitializeFromRobot
   （或等价预填）提供；未初始化返回 ROBOT_TARGET_NOT_INITIALIZED(-107)。
   腰 conf 缺省时按 NIE 1-DOF 语义补标（真实 dualarm conf 可能无 body unit）。
   NIE 运动学参数改 confbase 驱动（kinematic_params 注入，空则保留
   nie_arm_conf.hpp 缺省）。
4. **错误码统一**：CNieTarget/CNieWaist/CNieDual 返回值 bool→int
   ROBOT_TARGET_* 码（正=奇异 warning，负=error，1=OK）；新增
   NOT_ATTACHED(-1)/NOT_INITIALIZED(-107)/JOINT_OUT_OF_RANGE(-108)/
   JOINT_4/6_UNCERTAIN(-125/-126)/VALID_JOINTS_NOT_FOUND(-127)/
   ERROR_MODEL_NOT_IMPL(-203)。CArmTarget 恢复真实码透传（删 18 处
   `?1:0` 折叠）；CKinGeneRbt 内部 ELBOW_* 码在公共边界翻译为
   ROBOT_TARGET_*（奇异 80-88→10-18，不可达 -1001~-1005→-101）；
   CKinPLinkRbt IRB_* 码数值本已对齐（-200 ERROR_MODEL_NOT_IMPL 改 -203
   避开 WEAVE_POSE_DEFINITION 撞号）。CNieWaist::SetWaistTheta 超限
   钳制并返回 -108（此前静默），限位可 SetLimits 配置。
5. **flavor 收口**：nie_*.cpp 仅编入 libkinematic_dualarm.a；action 的
   SyncNie*/IsBoundNieGroup、pathplanning/ppwodyn 的 NIE 块、
   kinplanning NIE backend 全部 #if ROBOT_TYPE 门控为 dualarm 专属；
   nm_flavor_check.cmake 新增 humanoid NIE 符号阴性守卫（五库双 flavor
   全过）。

⚠️ **ABI 破坏**（task 侧重链 + 代码适配）：
- `CNieGroup::Attach` bool→int；`CalcDualPose/CalcDualJoint/CalcDualJacobian`
  bool→int（负码=错误；`if(ret)` 判真会把负码当成功——必须改 `>0`/`<=0`）
- `CNieTarget::CalcPose/CalcJoint/CalcVelocityJacobian`、`CNieWaist::SetWaistTheta`
  （void→int）、`CNieDual::CalcJoint`（0→-101）同语义
- 底层求解器 `CKinGeneRbt/CKinPLinkRbt` 公共方法 bool→int
- `KinematicsResolver::AttachNieGroup` → `AttachGroup(CTargetGroup*)`（dualarm 别名过渡一轮）
- `CompositeRobotModel::Init(const CNieGroup&)` 重载删除（用 `const CTargetGroup&`）
- **dualarm 槽位语义变化**：`soc_enable[]`/TargetData 槽位按紧凑枚举
  （task 侧 soc=1 即左臂，与 task 现有用法一致——这正是需求本体）
- shm 结构不变（TargetGroupSharedData 布局未动）

⚠️ **行为变化**：CNieWaist 超 ±150°（或 SetLimits 值）从静默写入改为钳制
+ 返回 -108。

⚠️ **task 侧待办**：
1. 重链接全链，跑 `dualarm_manual_probe` 两探针（验收标准 1）
2. `AttachNieGroup` 别名窗口一轮后删除
3. 若 task 侧有 `if (ret)` 风格判断 Calc* 返回值的位置，改 `> 0`

回归证据：kinematic 29+25、kinplanning 8+7、action 9+9、ppwodyn 16+12、
pathplanning 27+22 全绿；nm 双向自检 + NIE 阴性守卫五库双 flavor 全过；
E2E PNG（path/speed/joint 曲线）重新生成。
既有缺口（非本批次引入）：pathplanning_humanoid 12 例中
test_pipeline/test_e2e_whole_body_plan 红灯（TOPP 力矩回退，疑 09-30
confbase flavor 批次遗留，当时未对该库回归）；建议单独排查。

# 追加批次（2026-09-30：servo JointToMotor dualarm 恢复 + confbase 链接名适配）

需求链：`docs/2026-09-29-func-joint-to-motor-dualarm.md`（task 联署）、
`docs/2026-09-29-confbase-dualarm-flavor-requirement.md`（§6 验收 2/4 本批次闭环）
前提：base 侧 confbase flavor 化交付（`bbt_base_lib/libconfbase_<flavor>.a`）
设计/计划：`docs/superpowers/specs/2026-09-30-func-joint-to-motor-restore-design.md`、
`docs/superpowers/plans/2026-09-30-func-joint-to-motor-restore.md`

变更摘要：
1. **servo 恢复**：`CRobotTransformer::JointToMotor` 删除 dualarm 降级分支，
   恢复统一实现——dualarm 下经静态转换桥（13→57 前缀填充，base 裁定保留）
   + confbase_dualarm 13 宽循环完成全局关节↔电机转换。降级用例删除，
   `JointToMotorAppliesMask`、`GenerateBrakingWithStoreTransformer` 放开守卫
   在 dualarm 实跑通过。
2. **链接名适配**：func 7 主库 + 相关 unittest 的 `confbase` 链接名改为
   `confbase_${FUNC_FLAVOR_DIR}`（dynamics 硬写 dualarm、dyn_humanoid/
   pathplanning_humanoid 硬写 humanoid）；dynamics/dyn_humanoid/
   pathplanning_humanoid 的 `libconfbase.a` EXISTS 守卫同步更新。
3. **交付范围**：仅 `libservo_humanoid.a` / `libservo_dualarm.a` 覆盖上架；
   其余库仅本地重建回归（.a 内容无实质变化，用户裁定不上架）。

⚠️ **task 侧注意**：base 已将 `libconfbase.a` 删除并改名
`libconfbase_<flavor>.a`——task 侧所有按裸名 confbase 链接的位置必须同步
修改（func 侧已全部改完）。task 侧移除 `servo_engine_test.cpp` 的
`SKIP_IF_JOINT_TO_MOTOR_UNAVAILABLE()` 宏即可恢复 11 例测试。

回归证据：servo 双 flavor 45/45；kinematic/action/kinplanning/ppwodyn/
pathplanning 双 flavor + dynamics/dyn_humanoid/pathplanning_humanoid/
action_fsm 全量 ctest 全绿。

---

# 追加批次（2026-09-29：servo/spline 布局护栏放开 dualarm + 双库 flavor 后缀化——破坏性变更）

需求：`docs/2026-09-29-func-servo-step-data-dualarm.md`（task 侧阻塞项）
设计/计划：`docs/superpowers/specs/2026-09-29-servo-spline-dualarm-flavor-design.md`、
`docs/superpowers/plans/2026-09-29-servo-spline-dualarm-flavor.md`

⚠️ **破坏性变更（承接 2026-09-27 命名后缀化批次）**：

1. **servo/spline 改名并后缀化**（旧文件已删除）：
   - `libservo.a` → `libservo_humanoid.a` / `libservo_dualarm.a`
   - `libspline.a` → `libspline_humanoid.a` / `libspline_dualarm.a`
   task 侧链接名同步修改（与 09-27 批次同模式）。

2. **头文件断言分支化**：`servo_step_data.hpp` 的 ServoStepData 布局断言、
   `spline/types.hpp` 的 JointArray 断言改为 `#if ROBOT_TYPE ==` 双 flavor 分支
   （HUMANOID 57 / DUAL_ARM 13 各自钉死，240B 电机步断言不变）。
   `ROBOT_TYPE=DUAL_ARM` 编译不再被拒。

3. **身份符号/自检**：两库均内置 `g_func_lib_flavor_<flavor>`，POST_BUILD nm
   双向自检，可用 `bbt_func_lib/cmake/verify_flavor.cmake` 在 configure 期校验。

⚠️ **dualarm 功能限制（重要）**：`libconfbase.a`（bbt_base）的 `CJointPos`
全数组转换按 57 宽循环（HUMANOID 锁），dualarm 13 宽数组会越界 segfault
（gdb 实证）。func 侧已优雅降级：`CRobotTransformer::JointToMotor` 在
dualarm 下 `Log_E` + 返回 false（**全局关节↔电机转换 dualarm 暂不可用**）。
confbase flavor 化需求已提交：`docs/2026-09-29-confbase-dualarm-flavor-requirement.md`。
`GetUnitJointContext` 返回的指针按下标访问（logical < 13）安全，但不得用于
构造 CJointPos 调用全数组转换方法。

范围说明：spline 后缀化超出 task 文档字面（文档只提 servo）——servo 源码
include `spline/types.hpp`（57 断言同样阻塞编译），且 `spline::JointArray`
跨库 ABI 依赖关节数，spline 是 func 侧编译闭环的必然部分。

回归证据：spline 双 flavor ctest 90/90 全绿；servo humanoid 45/45 全绿
（基线不瘦身）；servo dualarm 44/44 全绿。
servo dualarm 豁免用例（confbase 57 锁所致，confbase flavor 化后放开）：
- `PathStopAcceptance.GenerateBrakingWithStoreTransformer`（humanoid-only 守卫）
- `CRobotTransformerTest.JointToMotorAppliesMask` → dualarm 下替换为
  `JointToMotorDegradesGracefully`（验证优雅降级行为，非豁免）

---

# 追加批次（2026-09-27：func 库 flavor 命名后缀化 + 扁平交付——破坏性变更）

设计/计划：`docs/superpowers/specs/2026-09-27-func-lib-flavor-naming-design.md`、
`docs/superpowers/plans/2026-09-27-func-lib-flavor-naming.md`
task 侧适配交接：`docs/2026-09-27-func-lib-flavor-naming-task-handoff.md`

⚠️ **破坏性变更，task 侧必须同步修改链接配置**：

1. **库文件改名**：5 个双 flavor 库加 flavor 后缀，且 `humanoid/`、
   `dualarm/` 子目录已删除，全部扁平放在 `bbt_func_lib/` 根目录：

   | 旧路径 | 新路径 |
   |--------|--------|
   | `bbt_func_lib/humanoid/libkinematic.a` | `bbt_func_lib/libkinematic_humanoid.a` |
   | `bbt_func_lib/dualarm/libkinematic.a` | `bbt_func_lib/libkinematic_dualarm.a` |
   | `<flavor>/libaction.a` | `libaction_<flavor>.a` |
   | `<flavor>/libkinplanning.a` | `libkinplanning_<flavor>.a` |
   | `<flavor>/libppwodyn.a` | `libppwodyn_<flavor>.a` |
   | `<flavor>/libpathplanning.a` | `libpathplanning_<flavor>.a` |

2. **不改名库**（也去掉了目录前缀）：`libaction_fsm.a`、`libdynamics.a`、
   `libdyn_humanoid.a`、`libpathplanning_humanoid.a`、`libspline.a`、
   `libservo.a`、`libfbbase_igh.a`、`libfbbase_soem.a` 均在根目录。

3. **身份符号**：每个带后缀库内置 `g_func_lib_flavor_<flavor>` 符号，
   `nm lib<a>_<flavor>.a | grep g_func_lib_flavor` 可查身份。

4. **configure 期校验**（推荐接入）：

   ```cmake
   include(<path>/bbt_func_lib/cmake/verify_flavor.cmake)
   verify_func_flavor(<path>/bbt_func_lib/libkinematic_humanoid.a humanoid)
   ```

变更动机：同名库靠目录区分，链接错目录会静默成功、运行时才出错。
后缀使命名即身份，选错 flavor 在链接期即报"找不到文件"。

回归证据：func 侧 9 库全量重建，kinematic/action/kinplanning/ppwodyn/
pathplanning 双 flavor + action_fsm/dynamics/dyn_humanoid/
pathplanning_humanoid ctest 全绿；构建期 nm 双向自检（身份符号 +
DualArmConfigStore 泄漏）全过。

---

# 追加批次（2026-09-27：CTarget::_robot_param 未初始化缺陷修复 + kinematic 源码对齐）

缺陷报告：`docs/2026-09-24-func-armtarget-uninit-robot-param-defect.md`
设计/计划：`docs/superpowers/specs/2026-09-27-armtarget-robot-param-init-design.md`、
`docs/superpowers/plans/2026-09-27-armtarget-robot-param-init.md`
交付说明：`docs/2026-09-27-func-armtarget-robot-param-init-delivery.md`

变更摘要：
- kinematic：`target.hpp` 的 `_robot_param` 值初始化清零（FK 路径
  `ResolveRobotParam_` 分支判定不再读未初始化值，确定性走 conf/URDF 回退）。
- 同批：`func/kinematic` 源码与 `reference/kinematic_backup0923` 对齐
  （恢复本地可复现架上库能力），本地独占旧文件归档
  `reference/kinematic_local_presync_2026-09-27/`。
- 交付 flavor：**humanoid + dualarm 双 flavor**（libkinematic.a，
  POST_BUILD 已自动上架）。

⚠️ ABI：无破坏（sizeof/布局不变），但头文件变化——task 侧需**重编译**
（非仅重链接）；下游 func 库（kinplanning/ppwodyn/pathplanning）已全量
重编双 flavor 回归验证。
⚠️ 验收：task 侧 `MALLOC_PERTURB_=165 ./planning_runner_test` +
WaterLevel 前置顺序全量 + valgrind `CalcPoseNom` 路径复核（func 本机
无 valgrind，等效证据见交付说明）。

---

# 追加批次（2026-09-24 FR5：手指/拇指 SOC 进入规划全管线）

需求：`docs/2026-09-24-func-fr5-finger-thumb-pipeline-requirement.md`
设计/计划：`docs/superpowers/specs/2026-09-24-fr5-finger-thumb-pipeline-design.md`、
`docs/superpowers/plans/2026-09-24-fr5-finger-thumb-pipeline.md`
交付说明：`docs/2026-09-24-func-fr5-finger-thumb-pipeline-delivery.md`

变更摘要：
- kinplanning：CRM 门控推广到 `CThumbTarget`/`CGFingerTarget`（经
  `ClassifyKinematicModel` 三类集合；`SocDesc.kinematic_model_name` 填实际
  模型名）；`KinematicsResolver` 手指/拇指走自带 FK/IK（TB/GF，自包含，
  `rc >= 0`），不进手臂后端。
- ppwodyn：`RunAll` 丢弃检测（enable + 航点 ≥2 未纳入 → `kInvalidInput`
  显式报错带 soc 号，禁止静默丢弃）；手指/拇指 `ACTION_LINE` 显式硬报错
  `kUnsupportedMotionType`。
- 交付 flavor：**humanoid + dualarm 双 flavor**（kinplanning/ppwodyn 两库）。

⚠️ ABI：无破坏（仅 .cpp 内部逻辑），与 09-16/09-24 FR4 批次可混链；
task 侧重链接即可（先删旧二进制，陷阱 #4）。
⚠️ 行为变化：① 手指/拇指 `ACTION_JOINT` 由「必然规划失败进 ERROR」变为
可规划——task 侧 FR4 空窗期拦截（若有）按语义决定去留；② 组内含
「enable + 航点 ≥2 但未支持模型名」的 soc 时 RunAll 由静默丢弃变为
显式报错（带 soc 号）——混合组配置错误的暴露面变大，属需求 3 要求。
⚠️ conf 缺口：部署 cfg 手指/拇指 unit 无 `drivechain` 节（关节速度限值
来源），Step4 限幅对真机手指暂不生效，字段清单见交付说明 §conf 核查，
task 侧补齐。

---

# 追加批次（2026-09-24 FR4：CRM 透传 SOC 映射补齐）

需求：`docs/2026-09-21-func-fr4-crm-passthrough-soc-map-requirement.md`
设计/计划：`docs/superpowers/specs/2026-09-24-fr4-crm-soc-map-design.md`、
`docs/superpowers/plans/2026-09-24-fr4-crm-soc-map.md`
交付说明：`docs/2026-09-24-func-fr4-crm-soc-map-delivery.md`

变更摘要：
- kinplanning/ppwodyn/pathplanning 三份 `ToConfigSocType` 拷贝统一修复：
  humanoid flavor 恒等映射（SOC 槽位 0..14 全覆盖，`static_assert` 锁定
  「枚举值==槽位号」）；dualarm flavor 保留显式 switch，行为零变化。
  修复前 HEAD(12)/AGV_FOOT(14) 落 default→NO_UNIT→CRM 门控静默跳过
  （"no enabled SOC" 根因），ppwodyn Step3/Step4 静默退化。
- kinplanning 可观测性：门控跳过（store 查询失败，带 soc 号+模型名）与
  Init 成功汇总（enabled SOC 列表）新增 DEBUG 日志。
- 语义边界（README §2.3）：仅 HEAD/BODY/AGV 标 `joint_passthrough`；
  拇指/手指走 `CThumbTarget`/`CGFingerTarget` 专用通道（部署 conf 已落实）。
- 交付 flavor：**humanoid + dualarm 双 flavor**（kinplanning/ppwodyn/
  pathplanning 三库）。

⚠️ ABI：无破坏（仅 .cpp 内部逻辑），与 09-16 批次可直接混链；
task 侧重链接即可。注意外部 .a 不产生 make 依赖——重链前删除旧
测试/应用二进制。

---

# 追加批次（2026-09-16 FR1 GetL 钳制域修复 + FR2 关节透传 target）

驱动文档：`docs/2026-09-15-func-change-requests.md`（FR3 挂起未做）
设计/计划：`docs/superpowers/specs/2026-09-16-func-fr1-getl-clamp-fr2-joint-passthrough-design.md`、
`docs/superpowers/plans/2026-09-16-func-fr1-getl-clamp-fr2-joint-passthrough.md`
交付说明：`docs/2026-09-16-func-fr1-getl-clamp-delivery.md`、
`docs/2026-09-16-func-fr2-joint-passthrough-delivery.md`
task 侧交接：`docs/2026-09-16-func-fr1-fr2-task-handoff.md`

变更摘要：
- spline（FR1）：`PathSpline::GetL` 输出钳制域改为主样条自身值域
  `[y(0), y(delta_x)]`（现算，零 ABI），修复 09-10 R2 钳到 `[0, main_length]`
  的量纲回归；main_length 未设置时现在也钳制（旧"不钳"语义作废）。
- kinematic（FR2）：新增 `CJointPassthroughTarget` 纯关节透传 target，
  conf `kinematic_model: "joint_passthrough"` 显式驱动；`CTarget` 新增
  `GetActiveTargetData()`。
- action（FR2 连锁）：`CActionBase::SetStartTarget/SetEndTarget` 重写
  （`GetActiveTargetData()` + shm re-Attach 重建 + 保留 `_robot_param`）。
- kinplanning（FR2 连锁）：`CompositeRobotModel` 门控纳入透传 unit；
  `KinematicsResolver` 透传 FK/IK 短路。
- ppwodyn（FR2 连锁）：Step3 类型无关取数 + kJoint 端点继承起点身份字段；
  新增 BODY 透传 E2E（JSON+matplotlib）。
- 交付 flavor：**humanoid + dualarm 双 flavor**（kinematic/action/kinplanning/
  ppwodyn 四库）；spline 根目录中立库。

⚠️ ABI 破坏：`CTarget` 新增成员（sizeof 变化）→ kinematic/action/kinplanning/
ppwodyn 四库必须整批重链接，禁止新旧混链。`TargetData` 布局未变，shm 无需重建。

⚠️ 行为变化：① FR1——`main_length=0` 时 `GetL` 也钳制到主样条值域；
② FR2——`CalcPose` 对透传 unit 返回成功但位姿有效标志保持 false
（笛卡尔动作在规划层硬报错）；③ `CompositeRobotModel` 仅当 conf 显式配置
`joint_passthrough` 时才纳入 BODY/HEAD/AGV 等无模型 unit，未配置零变化。

task 侧适配：重链接 libspline.a + 整批 4 库；conf 为 BODY 加
`kinematic_model: "joint_passthrough"` + 关节限值；复测 `planning_runner_test`
（`ProcessSingleJointActionSucceeds`）与 BODY 实机探针。

---

# 追加批次（2026-09-02 joint 动作声明入口）

驱动文档：`docs/2026-09-01-func-joint-motion-entry-request.md`
设计/计划：`docs/superpowers/specs/2026-09-02-func-joint-motion-entry-design.md`、
`docs/superpowers/plans/2026-09-02-func-joint-motion-entry.md`

变更摘要：
- kinematic：`TargetData` 新增 `ACTION_MOTION_TYPE motion_type` 字段；
  `ACTION_MOTION_TYPE` 枚举自 action 下沉至 `legacy_robot_types.hpp`；
  `CBaseTarget`/`CTarget` 新增 `SetMotionType/GetMotionType`。
- action：枚举改 include kinematic 头（源兼容，宏保留）。
- ppwodyn：`RunAll` 按终点 `motion_type` 分派（LINE/JOINT），不再硬编码
  ACTION_LINEAR；NULL 端点压缩（保持位置、不规划、不下发）；多 run 拼接补
  `merged.a`/`is_joint` 列（混合序列可进 phase2）；joint 样本时间缩放跳过
  笛卡尔约束；fallback profile 时跳过 phase2 并告警。
- 交付 flavor：**humanoid**（dualarm 本机验证通过，不作为本批交付物宣布）。

⚠️ ABI 破坏：`TargetGroupSharedData` 布局变化（TargetData 尾部新增字段）。
task 侧 shm 相关区域需随新版本重建，全部消费方必须重编。

⚠️ 行为变化：`motion_type` 零值（memset 0）= ACTION_NULL = **无运动**
（该 SOC 不规划、不下发）。调用方必须显式填写 ACTION_LINE/ACTION_JOINT，
不再存在"默认按直线规划"。

task 侧适配（对方承诺，见需求文档 §5）：
`PlanningRunner::BuildWaypointGroups` 按协议透传 motion_type。

---

# func 层库交付说明（2026-08-24 重交付）

驱动文档：`docs/2026-08-24-func-libs-humanoid-redelivery-request.md`
设计/计划：`docs/superpowers/specs/2026-08-24-func-libs-dual-flavor-redelivery-design.md`、`docs/superpowers/plans/2026-08-24-func-libs-dual-flavor-redelivery.md`

# 追加批次（2026-08-26 反馈收编）

驱动文档：`docs/2026-08-26-ppwodyn-kinematic-integration-feedback.md`
设计/计划：`docs/superpowers/specs/2026-08-26-ppwodyn-kinematic-feedback-design.md`、
`docs/superpowers/plans/2026-08-26-ppwodyn-kinematic-feedback.md`
总结：`docs/2026-08-26-ppwodyn-kinematic-feedback-summary.md`

变更摘要：
- kinematic：`InitializeFromRobot` 配置缺失时不再擦除已有数据（修复 task 反馈 #1，
  真凶是无条件 memset 而非 Detach）；无运动学映射槽位日志 ERROR→DEBUG 并修复
  日志恒打 -1 的值 bug（反馈 #6；含 GetJointNumber 同类第三处）。
- kinplanning：**ABI 破坏**——`SegmentEndpointDynamics` 关节数组定宽 6→7
  （`SegmentJointArray`，反馈 #3）；`PathActionSegment` 字段注释按实测语义修正
  （反馈 #2）。下游必须重编。
- pathplanning：编译适配（动力学路径保持 6-DOF 作用域，显式取前 6 关节）；
  `kMaxSegmentDof` 同源拓宽。
- ppwodyn：第 7 关节漏管修复（kMaxSegmentDof + profiler 关节循环/类型拓宽）；
  Step6 加速度校验量纲修正（反馈 #4 误报消除）；fallback 路径 `v_*` 统一为
  归一化 L 域 dL/dt；README 补限幅来源/告警语义/起点奇异说明（反馈 #4/#5）。

task 侧注意：**全部 5 库需重编**（kinplanning ABI 变化）；task 转换层只读
`*_target_data.joints`（7 宽），与新结构体兼容，零改动受益。

遗留已知项（本批次未动）：
- pathplanning 的 Step6 存在与 ppwodyn 同源的量纲问题，本次未修（pathplanning
  v_* 语义经同步阶段后为物理域，与 ppwodyn 不同；如需收编请单提）。
- profiler 对 `dq_dL`（action 级差分）按段折算在多子段时偏保守，现状文档化
  （kinplanning README §2.1）。

## 构建 flavor

| 库 | bbt_func_lib/ 根 | humanoid/ | dualarm/ | kRobotMaxJoints |
|---|---|---|---|---|
| libkinematic.a | —（2026-08-27 起不再放） | HUMANOID | DUAL_ARM | 57 / 13 |
| libaction.a | —（同上） | HUMANOID | DUAL_ARM | 57 / 13 |
| libkinplanning.a | —（同上） | HUMANOID | DUAL_ARM | 57 / 13 |
| libppwodyn.a | —（同上） | HUMANOID | DUAL_ARM | 57 / 13 |
| libpathplanning.a | —（同上） | HUMANOID | DUAL_ARM | 57 / 13 |
| libdynamics.a | —（不交付） | —（不交付） | DUAL_ARM | 13 |

- **2026-08-27 起（用户要求）：根目录不再接收 func 库拷贝**——各库 CMake POST_BUILD
  的"HUMANOID 同步根目录"步骤已删除，只部署到 `bbt_func_lib/<flavor>/`；dynamics
  由根目录直拷改为 `dualarm/`。消费方链接路径：`bbt_func_lib/humanoid/`（交付 flavor）
  或 `bbt_func_lib/dualarm/`。根目录既有的 lib{kinematic,action,kinplanning,
  pathplanning,ppwodyn}.a 已于 2026-08-27 经用户确认删除（最后一批根拷贝为
  2026-08-26 版本）。
- HUMANOID 版 libpathplanning 不含动力学功能（`InitDualArm`/`DualArmSynchronizer`/`UnifiedDynamicsModel`/`MultiSocCoordinator` phase2/`TimeOptimalProfiler`/`FillTorqueLimits13`/Step7 关节动力学富化等均已 `#if ROBOT_TYPE == DUAL_ARM` 切除）；纯运动学规划完整（几何相位 + IK + 速度检查 + 梯形兜底 + MultiArmSynchronizer)。
- libdynamics.a 的 HUMANOID 版**未交付**;task 侧接入动力学规划前需先提出 HUMANOID dynamics 需求。
- libspline.a、libfbbase_*.a flavor 中立，沿用原交付（仍在根目录）。

## 自检结果（交付前已跑）

`nm -C bbt_func_lib/lib<name>.a | grep -c DualArmConfigStore`:

| 库 | root | humanoid/ | dualarm/ |
|---|---|---|---|
| libkinematic.a | 0 | 0 | 10 |
| libaction.a | 0 | 0 | 2 |
| libkinplanning.a | 0 | 0 | 4 |
| libppwodyn.a | 0 | 0 | 7 |
| libpathplanning.a | 0 | 0 | 11 |

nm 自检已固化进各库 CMake POST_BUILD(HUMANOID 构建时自动执行，符号非零即 build 失败）。

## ctest（双 flavor 全绿）

| 库 | dualarm | humanoid |
|---|---|---|
| kinematic | 21/21 | 21/21 |
| action | 8/8 | 8/8 |
| kinplanning | 5/5 | 5/5 |
| ppwodyn | 11/11 | 11/11 |
| pathplanning | 27/27 | 27/27 |

说明：dualarm 专属测试（动力学/dualarm 配置绑定）以 `#if ROBOT_TYPE == DUAL_ARM` 保留源码，HUMANOID 构建下为 0-test 二进制（ctest 通过不计用例）;HUMANOID 侧新增 4 个冒烟/E2E 测试（store attach、CompositeRobotModel 双臂发现、ppwodyn 单臂单段 E2E、pathplanning 纯运动学限幅，均含性能断言与 JSON+matplotlib 出图）。

## 构建方式

```bash
cmake -S func/<lib> -B func/<lib>/build_humanoid -DROBOT_TYPE=HUMANOID
cmake -S func/<lib> -B func/<lib>/build_dualarm  -DROBOT_TYPE=DUAL_ARM
cmake --build func/<lib>/build_<flavor> -j
```

ROBOT_TYPE 未显式指定时 configure 直接 FATAL_ERROR(func 侧的 DUAL_ARM 默认值陷阱已消除；confbase/robot.hpp 的静默 HUMANOID 默认不再有机会造成混合 flavor)。

注意：func/dynamics 未做本次 CMake 改造（按其现状 DUAL_ARM 默认值构建归位 dualarm/),HUMANOID dynamics 需要时需另行适配。

## 2026-08-25 更新：基于 confbase 2026-08-24 版（kShmVersion=5)

- func 侧临时方案（`robot_store_selector.hpp`/`robot_type_macros.hpp`、`KinematicsResolver` 双 flavor Init 重载）已回退删除，统一使用 `confbase/robot.hpp` 的 `RobotConfigStore`/`RobotSocUnitType` 别名（泛型 store 冲突已由 confbase 根治：adapter 头直接包含 robot.hpp，泛型 Attach 重载已移除）。
- UnitConfig 新字段消费路径（`KinematicConfigAdapter::GetKinematicModelName`、target.cpp 模型选择）已验证；真实 builder 填充 probe 结果：`RIGHT_ARM/LEFT_ARM` 单元 `model='CKinGeneRbt' root='Body' tip='Rarm_ter'/'Larm_ter' joints=7`(task 侧 "model ''" 告警根治证据）。
- test helper patch 清理实验（2026-08-25):4 个 patch(PatchRobotTopology / PatchDriveChainTransmissionRatios / PatchJointDriveSpeedLimits / PatchKinematicMdhParams）经逐一移除实验证实**仍全部需要**（移除分别导致 3/7/2/2 个 dualarm 测试失败），已恢复并在 helper 中注释标注。
- ctest 双 flavor 全绿：kinematic 21、action 8、kinplanning 5、ppwodyn 11、pathplanning 27;dynamics DUAL_ARM 16/16（仅回归，未改动）。
