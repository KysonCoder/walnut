#ifndef TASK_SERVO_SERVO_STEP_HPP_
#define TASK_SERVO_SERVO_STEP_HPP_

#include "servo_step_data.hpp"
#include "confbase/robot.hpp"
#include "confbase/joint_pos.hpp"

namespace servo {

class ServoStep {
public:
    ServoStep();
    explicit ServoStep(ServoStepData* external_data);
    ~ServoStep();

    ServoStep(const ServoStep& other);
    ServoStep& operator=(const ServoStep& other);

    void AttachTo(ServoStepData* external_data);
    void Detach();

    ServoStepData* GetData() { return data_; }
    const ServoStepData* GetData() const { return data_; }

    void Clear();

    // unit/global_map 经 RobotTransformer::GetUnitJointContext 获取；
    // 任一为空则记日志并直接返回。pos_motor 等 CJointPos 必须带
    // GlobalJointMap 构造（逻辑访问需要）。
    void SetPosSpeedAcc(
        const robot_config::UnitConfig* unit,
        const robot_config::GlobalJointMap* global_map,
        const CJointPos* pos_motor,
        const CJointPos* speed_motor,
        const CJointPos* acc_motor);

    void GetPosM(const robot_config::UnitConfig* unit,
                 const robot_config::GlobalJointMap* global_map,
                 CJointPos* pos_out);

    void GetSpeedM(const robot_config::UnitConfig* unit,
                   const robot_config::GlobalJointMap* global_map,
                   CJointPos* speed_out);

    void GetAccM(const robot_config::UnitConfig* unit,
                 const robot_config::GlobalJointMap* global_map,
                 CJointPos* acc_out);

    ServoStepMotorData* GetMotorStep(int joint);
    const ServoStepMotorData* GetMotorStep(int joint) const;
    ServoStepMotorData* GetAllMotorSteps();
    const ServoStepMotorData* GetAllMotorSteps() const;

private:
    ServoStepData own_data_;
    ServoStepData* data_;
};

}  // namespace servo

#endif
