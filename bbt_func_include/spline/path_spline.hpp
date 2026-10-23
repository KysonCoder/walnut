#ifndef SPLINE_PATH_SPLINE_HPP_
#define SPLINE_PATH_SPLINE_HPP_

#include "spline/spline.hpp"
#include "spline/joint_spline.hpp"
#include "spline/spline_data.hpp"

namespace spline {

class PathSpline {
public:
    PathSpline();
    explicit PathSpline(PathSplineData* external_data);
    PathSpline(const PathSpline& other);
    PathSpline& operator=(const PathSpline& other);

    void AttachTo(PathSplineData* external_data);
    void Detach();

    void clear();

    CSpline& main_spline() { return main_spline_; }
    const CSpline& main_spline() const { return main_spline_; }
    JointSpline& joint_spline() { return joint_spline_; }
    const JointSpline& joint_spline() const { return joint_spline_; }

    double GetMainLength() const;
    void SetMainLength(double length);

    double GetJointLength() const;

    double GetPathTime() const;
    void SetPathTime(double t);

    bool GetL(double t, double& l) const;
    bool GetDldt(double t, double& v) const;
    bool GetD2ldt2(double t, double& a) const;

    // SOC 路径长度管理
    double GetSocPathLength(int soc_idx) const;
    void SetSocPathLength(int soc_idx, double l);

    // L 映射计算
    double GetL(int soc_idx, double s) const;
    double GetdLds(int soc_idx) const;

    // 对 L 的导数（链式法则包装）
    bool EvalDerivativeWrtL(double s, int soc_idx, JointArray& dq_dL);
    bool EvalSecondDerivativeWrtL(double s, int soc_idx, JointArray& d2q_dL2);

    bool IsEmpty() const;

    PathSplineData* GetData() { return data_; }
    const PathSplineData* GetData() const { return data_; }

private:
    void BindSplinesToData();

    PathSplineData own_data_;
    PathSplineData* data_;
    CSpline main_spline_;
    JointSpline joint_spline_;
};

} // namespace spline

#endif // SPLINE_PATH_SPLINE_HPP_
