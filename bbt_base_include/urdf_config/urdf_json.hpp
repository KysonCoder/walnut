#ifndef URDF_CONFIG_URDF_JSON_HPP
#define URDF_CONFIG_URDF_JSON_HPP

#include "urdf_config/urdf_types.hpp"
#include <nlohmann/json.hpp>

namespace urdf {

nlohmann::json to_json(const Robot& robot);

} // namespace urdf

#endif // URDF_CONFIG_URDF_JSON_HPP
