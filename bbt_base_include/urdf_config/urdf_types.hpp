#ifndef URDF_CONFIG_URDF_TYPES_HPP
#define URDF_CONFIG_URDF_TYPES_HPP

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace urdf {

struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Origin {
    std::optional<Vector3> xyz;
    std::optional<Vector3> rpy;
};

struct Inertia {
    double ixx = 0.0;
    double ixy = 0.0;
    double ixz = 0.0;
    double iyy = 0.0;
    double iyz = 0.0;
    double izz = 0.0;
};

struct Inertial {
    std::optional<Origin> origin;
    std::optional<double> mass;
    std::optional<Inertia> inertia;
};

struct MeshInfo {
    std::string filename;
    std::optional<Vector3> scale;
    std::optional<Origin> origin;
};

struct Link {
    std::string name;
    std::optional<Inertial> inertial;
    std::vector<MeshInfo> visual_meshes;
    std::vector<MeshInfo> collision_meshes;
};

struct JointLimit {
    std::optional<double> lower;
    std::optional<double> upper;
    std::optional<double> effort;
    std::optional<double> velocity;
};

struct Joint {
    std::string name;
    std::string type;
    std::optional<std::string> parent;
    std::optional<std::string> child;
    std::optional<Origin> origin;
    std::optional<Vector3> axis;
    std::optional<JointLimit> limit;
};

struct Robot {
    std::string robot_name;
    std::string source_urdf;
    std::size_t link_count = 0;
    std::size_t joint_count = 0;
    std::vector<std::string> root_links;
    std::vector<Link> links;
    std::vector<Joint> joints;
    std::string raw_urdf_xml;
};

} // namespace urdf

#endif // URDF_CONFIG_URDF_TYPES_HPP
