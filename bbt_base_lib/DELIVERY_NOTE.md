# bbt_base_lib 交付说明 — confbase 双 flavor 批次（2026-09-30）

## 批次内容

- `libconfbase.a`（未分 flavor）**已废弃删除**，替换为：
  - `libconfbase_humanoid.a`（`-DROBOT_TYPE=HUMANOID` 构建，`CJointPos` 全数组方法按 57 宽）
  - `libconfbase_dualarm.a`（`-DROBOT_TYPE=DUAL_ARM` 构建，`CJointPos` 全数组方法按 13 宽）
- 头文件 `bbt_base_include/confbase/` 两 flavor 共用单份；`robot.hpp` 不再提供 `ROBOT_TYPE` 默认值，下游必须显式 `-DROBOT_TYPE=...`，否则编译期 `#error`
- 每库内置 flavor 身份符号 `g_func_lib_flavor_<flavor>`（nm 可查），POST_BUILD 已做阳性/阴性 nm 自检

## shm 布局声明

**shm 总线布局零变化**：`shm_layout.hpp` 通用类型（`GlobalJointMap`/`GlobalJointParams`/`RobotTopology` 等）保持 57 宽（`kMaxNoOfJoints=57`），`kShmVersion` 未升，全部 `static_assert(sizeof(...))` 数值不变。变化的只有 `CJointPos` 全数组方法的循环宽度（编译期 flavor 常量 `kRobotMaxJoints`）。

## 下游适配要求

- 链接名：`confbase` → `confbase_humanoid` 或 `confbase_dualarm`（func 侧按 `confbase_${FUNC_FLAVOR_DIR}` 批量适配，task 侧同理）
- base 内部消费方已适配：`base/drivechain/unittest/CMakeLists.txt` 链接名 `confbase` → `confbase_humanoid`（其仅用 flavor 无关的 `RobotConfigBuilder/Store` 通用 API，选 humanoid 作规范 flavor）；其余 base 库无 confbase 链接依赖
- func 侧恢复点：按 `docs/2026-09-29-confbase-dualarm-flavor-requirement.md` §7 清单执行（删除 JointToMotor 降级分支、放开两个测试守卫等），本批次已解除其前提
- task 侧恢复点：移除 `servo_engine_test.cpp` 的 `SKIP_IF_JOINT_TO_MOTOR_UNAVAILABLE()` 宏

## 构建方式

```bash
cd base/confbase
./build_all_flavors.sh   # 两 flavor 依次 configure + build + ctest
```

或单 flavor：`cmake -S . -B build/<flavor> -DROBOT_TYPE=HUMANOID|DUAL_ARM`（缺失/非法即 FATAL_ERROR）。

## §3.2 连带排查结论（关闭需求 §3.2/§5.3）

| 库 | 结论 | 证据 |
|----|------|------|
| libdrivechain.a | 无 ROBOT_TYPE/kMaxNoOfJoints/kRobotMaxJoints 敏感代码 | `grep -rl "ROBOT_TYPE\|kMaxNoOfJoints\|kRobotMaxJoints" base/drivechain/{src,include}` 无命中 |
| liburdf_config.a | 同上 | 同上（base/urdf_config 无命中） |
| libmath.a / libosbase.a / libutilbase.a | 无关（与需求预判一致） | 同上 grep 模式 |
| RobotConfigStore shm 解析 / Builder 宏路径 | 通用类型保持 57，无需改动；两 flavor 库均编译全部 builder/store 源码（func 版 nm 自检第 3 条对 confbase 不适用，见 spec §3.6） | 代码审查 |

## 验证

- `build_all_flavors.sh` 两 flavor 全量 ctest 通过（各 404/404，含新增 `test_flavor_width.cpp` 哨兵测试：DUAL_ARM 下证明 `CJointPos` 全数组方法不越出 13 宽）
- 其余 base 库全量重建回归（2026-09-30）：utilbase 77/77、math 245/245、osbase 1/1、urdf_config 16/16、drivechain 15/15
- spec：`docs/superpowers/specs/2026-09-30-confbase-dualarm-flavor-design.md`；plan：`docs/superpowers/plans/2026-09-30-confbase-dualarm-flavor.md`
