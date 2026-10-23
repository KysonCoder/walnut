#pragma once

#include <string>

class ConfigPathResolver {
public:
    // 解析 URDF、机器人配置与 servo_conf 路径。优先级：
    //   1. 路径配置文件中的 urdf=/robot_cfg=/servo_conf= 值
    //      （默认查找 ./cfg/taskframe_paths.cfg，可用重载显式指定）
    //   2. 内置默认（随编译期 ROBOT_TYPE 切换 flavor）：
    //      humanoid: /home/user/claude/cfg/humanoid.urdf、
    //                humanoid_robot_conf.cfg、servo_conf.cfg
    //      dualarm:  /home/user/claude/cfg/2arm_v1.urdf、
    //                dualarm_robot_conf.cfg、servo_conf_dualarm.cfg
    // urdf/robot_cfg 对应文件不存在时返回 false；
    // servo_conf 文件不存在仅告警（Fieldbus 有独立降级路径），不影响返回值。
    static bool Resolve(std::string& urdf_path, std::string& cfg_path,
                        std::string& servo_conf_path);
    static bool Resolve(const std::string& config_file,
                        std::string& urdf_path, std::string& cfg_path,
                        std::string& servo_conf_path);

    // 进程 exe 所在目录（/proc/self/exe；失败返回空串）。
    // 恢复文件等 app 同级数据目录的公共解析点（2026-10-13）。
    static std::string ResolveExeDir();
};
