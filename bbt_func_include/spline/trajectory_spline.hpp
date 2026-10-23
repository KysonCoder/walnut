#ifndef SPLINE_TRAJECTORY_SPLINE_HPP_
#define SPLINE_TRAJECTORY_SPLINE_HPP_

#include "spline/types.hpp"
#include "spline/joint_spline.hpp"

// 前向声明CAction
class CAction;

namespace spline {

// 速度限制条件 - 无CRobot依赖
struct SpeedCondition {
    CSpline lv_l;              // 路径长度映射样条
    double max_speed_start = 0.0;
    double max_speed_end = 0.0;
    double l_switch = 0.0;
    int motion_group = 0;
    enum class Type { TCP, ORIENTATION, EXTERNAL, NONE } type = Type::NONE;
};

// CAction的轻量级只读引用
class ActionRef {
public:
    explicit ActionRef(const CAction* action) : action_(action) {}

    bool IsValid() const { return action_ != nullptr; }
    const CAction* Get() const { return action_; }

    // 提取路径长度等信息的方法
    double GetPathLength() const;

private:
    const CAction* action_ = nullptr;
};

class TrajectorySpline {
public:
    TrajectorySpline();
    ~TrajectorySpline();

    // 从CAction创建 - 使用ActionRef包装，无CRobot依赖
    bool CreateFromAction(const ActionRef& action1,
                          const ActionRef& action2,
                          double delta_l);

    bool CreateZeroMove(const JointArray& q, double zero_move_time);

    // 速度条件管理
    bool AddSpeedCondition(const SpeedCondition& condition);
    void SetProcessSpeed(int motion_group, double speed, double l);

    // 速度限制计算
    bool GetSpeedLimit(double l, double& v_limit);
    double GetMinTcpSpeed(double l);

    // 评估 - 委托给JointSpline
    bool Eval(double l, JointArray& q);
    bool EvalDerivative(double l, JointArray& dq);
    bool EvalSecondDerivative(double l, JointArray& d2q);
    bool EvalThirdDerivative(double l, JointArray& d3y);

    // 系数提取
    void GetCoeffFromDerivative(JointArray& dc0,
                                JointArray& dc1,
                                JointArray& dc2);
    void GetCoeffFromSecondDerivative(JointArray& d2c0,
                                      JointArray& d2c1);

    // 访问器
    double GetLength() const;
    CSpline* GetLqLSpline();
    CSpline* GetLcLSpline(int motion_group);

    // 运动属性
    bool IsZeroMove() const { return is_zero_move_; }
    double GetZeroMoveTime() const { return zero_move_time_; }
    bool IsFinepoint() const { return is_finepoint_; }

private:
    static constexpr int kMaxSpeedConditions = 3;

    double delta_l_ = -1.0;
    JointSpline joint_spline_;
    CSpline lq_l_;  // 位置-长度映射

    SpeedCondition speed_conditions_[kMaxSpeedConditions];
    int num_speed_conditions_ = 0;

    // 运动属性
    bool is_zero_move_ = false;
    double zero_move_time_ = 0.0;
    bool is_finepoint_ = false;
};

} // namespace spline

#endif // SPLINE_TRAJECTORY_SPLINE_HPP_
