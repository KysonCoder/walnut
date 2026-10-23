#ifndef BASE_CONFBASE_INCLUDE_DYN_GROUP_MODEL_HPP_
#define BASE_CONFBASE_INCLUDE_DYN_GROUP_MODEL_HPP_

#include <cstdint>

namespace robot_config {

constexpr int32_t kDynGroupModelNoModel = -1;
constexpr int32_t kDynGroupModelDynMia = 0;
constexpr int32_t kDynGroupModelDynRx1 = 1;

bool StringToDynGroupModel(const char* name, int32_t* out_model);
const char* DynGroupModelToString(int32_t model);

}  // namespace robot_config

#endif  // BASE_CONFBASE_INCLUDE_DYN_GROUP_MODEL_HPP_
