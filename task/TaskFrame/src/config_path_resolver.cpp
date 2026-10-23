#include "config_path_resolver.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

#include "utilbase/logger.h"

static std::string Trim(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
        ++start;
    }
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }
    return s.substr(start, end - start);
}

static bool IsRegularFile(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

bool ConfigPathResolver::Resolve(std::string& urdf_path, std::string& cfg_path) {
    return Resolve("./cfg/taskframe_paths.cfg", urdf_path, cfg_path);
}

bool ConfigPathResolver::Resolve(const std::string& config_file,
                                 std::string& urdf_path, std::string& cfg_path) {
    urdf_path = "/home/user/claude/cfg/humanoid.urdf";
    cfg_path = "/home/user/claude/cfg/humanoid_robot_conf.cfg";

    std::ifstream file(config_file);
    if (!file.is_open()) {
        Log_D("ConfigPathResolver: %s not found, using defaults\n", config_file.c_str());
    } else {
    std::string line;
    bool any_key_read = false;
    bool urdf_from_config = false;
    bool cfg_from_config = false;
    while (std::getline(file, line)) {
        std::string trimmed = Trim(line);

        if (trimmed.size() > 4096) {
            Log_W("ConfigPathResolver: line exceeds 4096 characters, skipping\n");
            continue;
        }

        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

            size_t eq = trimmed.find('=');
            if (eq == std::string::npos) {
                continue;
            }

            std::string key = Trim(trimmed.substr(0, eq));
            std::string value = Trim(trimmed.substr(eq + 1));

            if (value.empty()) {
                Log_W("ConfigPathResolver: empty value for key '%s', ignoring\n", key.c_str());
                continue;
            }

            if (key == "urdf") {
                if (urdf_from_config) {
                    Log_W("ConfigPathResolver: duplicate 'urdf' key, overriding previous value\n");
                }
                urdf_path = value;
                urdf_from_config = true;
                any_key_read = true;
            } else if (key == "robot_cfg") {
                if (cfg_from_config) {
                    Log_W("ConfigPathResolver: duplicate 'robot_cfg' key, overriding previous value\n");
                }
                cfg_path = value;
                cfg_from_config = true;
                any_key_read = true;
            }
        }

        if (any_key_read) {
            Log_D("ConfigPathResolver: config file overrides defaults: urdf=%s cfg=%s\n",
                  urdf_path.c_str(), cfg_path.c_str());
        }
    }

    if (!IsRegularFile(urdf_path)) {
        Log_E("ConfigPathResolver: URDF file not found or not a regular file: %s\n", urdf_path.c_str());
        return false;
    }
    if (!IsRegularFile(cfg_path)) {
        Log_E("ConfigPathResolver: robot config file not found or not a regular file: %s\n", cfg_path.c_str());
        return false;
    }

    Log_D("ConfigPathResolver: using urdf=%s cfg=%s\n",
          urdf_path.c_str(), cfg_path.c_str());
    return true;
}
