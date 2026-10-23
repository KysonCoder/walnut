#ifndef URDF_CONFIG_URDF_DH_CONVERTER_HPP
#define URDF_CONFIG_URDF_DH_CONVERTER_HPP

#include "urdf_config/urdf_types.hpp"
#include <vector>

namespace urdf {

/// DH 参数约定类型
enum class DHConvention {
    Standard,  ///< 标准 DH (坐标系建在连杆近端)
    Modified   ///< 改进 MDH / Craig 约定 (坐标系建在连杆末端)
};

/// 统一的 DH 参数表
struct DHTable {
    std::vector<double> a;       ///< 连杆长度 a_i
    std::vector<double> alpha;   ///< 连杆扭角 α_i (rad)
    std::vector<double> d;       ///< 连杆偏距 d_i
    std::vector<double> theta;   ///< 关节角固定偏移 θ_i (rad)
    DHConvention convention;     ///< 使用的约定
    std::string robot_name;      ///< 机器人名称
};

/// 从 URDF 提取 DH 参数表
/// @param robot 已解析的 URDF 结构
/// @param convention DH 约定类型
/// @return DH 参数表
/// @note 要求 joints 按从 base 到 tip 的拓扑顺序排列
DHTable extractDHFromUrdf(const Robot& robot, DHConvention convention);

/// 从 DH 参数表生成 URDF Robot 结构
/// @param dh DH 参数表
/// @param robot_name 生成的机器人名称
/// @return URDF Robot 结构（仅包含 links 和 joints）
Robot buildUrdfFromDH(const DHTable& dh, const std::string& robot_name);

/// ABB IRB2400 MDH 参数（Craig 约定）
DHTable MakeIRB2400MDH();

/// ABB IRB2400 标准 DH 参数
DHTable MakeIRB2400StandardDH();

} // namespace urdf

#endif // URDF_CONFIG_URDF_DH_CONVERTER_HPP
