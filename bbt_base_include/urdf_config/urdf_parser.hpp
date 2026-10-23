#ifndef URDF_CONFIG_URDF_PARSER_HPP
#define URDF_CONFIG_URDF_PARSER_HPP

#include "urdf_config/urdf_types.hpp"
#include <string>

namespace urdf {

Robot parse_urdf(const std::string& urdf_path);
Robot parse_urdf_from_string(const std::string& xml_content,
                              const std::string& source_name = "");

} // namespace urdf

#endif // URDF_CONFIG_URDF_PARSER_HPP
