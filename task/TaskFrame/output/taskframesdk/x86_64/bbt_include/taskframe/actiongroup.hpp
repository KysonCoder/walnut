#ifndef __ACTIONGROUP_HPP__
#define __ACTIONGROUP_HPP__

// shm 环形队列元素：真实动作组 POD（func/action 库）。
// actiondata.hpp 内部扁平包含 legacy_robot_types.hpp / targetgroup.hpp /
// pose.hpp，分别解析到 bbt_func_include/kinematic/ 与 bbt_base_include/math/
// （targetgroup.hpp 若先命中本目录的同名头，由该头转发到真实版本，见
// targetgroup.hpp 的注释）。
#include "action/actiondata.hpp"

// Behavior wrapper kept for existing consumers (e.g., StateMaster).
// Methods operate on the underlying ActionGroupSharedData.
class CActionGroup {
public:
    CActionGroup() {}
    ~CActionGroup() {}

    void GetMasterPathLength(double* length) { *length = 1.17; }
    void SetStartTargets(CTargetGroup& target) {}

    template <typename T>
    bool SetData(T program_line) { return true; }

    const ActionGroupSharedData& GetSharedData() const { return data_; }
    ActionGroupSharedData& GetSharedData() { return data_; }

private:
    ActionGroupSharedData data_;
};

#endif
