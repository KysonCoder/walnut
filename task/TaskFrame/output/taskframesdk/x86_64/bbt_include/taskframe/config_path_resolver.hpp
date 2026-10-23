#pragma once

#include <string>

class ConfigPathResolver {
public:
    // 解析 URDF 与机器人配置路径。优先级：
    //   1. 路径配置文件中的 urdf=/robot_cfg= 值
    //      （默认查找 ./cfg/taskframe_paths.cfg，可用重载显式指定）
    //   2. 内置默认 /home/user/claude/cfg/humanoid.urdf 与
    //      /home/user/claude/cfg/humanoid_robot_conf.cfg
    // 解析结果对应文件不存在时返回 false。
    static bool Resolve(std::string& urdf_path, std::string& cfg_path);
    static bool Resolve(const std::string& config_file,
                        std::string& urdf_path, std::string& cfg_path);
};
