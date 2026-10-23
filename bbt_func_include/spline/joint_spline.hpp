#ifndef SPLINE_JOINT_SPLINE_HPP_
#define SPLINE_JOINT_SPLINE_HPP_

#include "spline/types.hpp"
#include "spline/spline.hpp"
#include "spline/spline_data.hpp"
#include <array>

namespace spline {

using CSpline = interpolation::CSpline;

class JointSpline {
public:
    JointSpline();
    explicit JointSpline(JointSplineData* external_data);
    JointSpline(const JointSpline& other);
    JointSpline& operator=(const JointSpline& other);

    void AttachTo(JointSplineData* external_data);
    void Detach();

    void clear();
    bool CreateComplete(double delta_x,
                        const JointArray& y1,
                        const JointArray& y2,
                        const JointArray& dy1,
                        const JointArray& dy2);
    bool CreateConstantSpline(double delta_x, const JointArray& q);

    bool Eval(double x, JointArray& y) const;
    bool EvalDerivative(double x, JointArray& dy) const;
    bool EvalSecondDerivative(double x, JointArray& d2y) const;
    bool EvalThirdDerivative(double x, JointArray& d3y) const;

    void GetCoeffFromDerivative(JointArray& dc0,
                                JointArray& dc1,
                                JointArray& dc2);
    void GetCoeffFromSecondDerivative(JointArray& d2c0,
                                      JointArray& d2c1);

    double GetLength() const;
    bool IsEmpty() const;

    JointSplineData* GetData() { return data_; }
    const JointSplineData* GetData() const { return data_; }

private:
    void BindSplinesToData();

    JointSplineData own_data_;
    JointSplineData* data_;
    CSpline splines_[robot_config::kRobotMaxJoints];
};

} // namespace spline

#endif // SPLINE_JOINT_SPLINE_HPP_
