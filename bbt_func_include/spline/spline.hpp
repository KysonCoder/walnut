#ifndef _ROB_SPLINE_H_
#define _ROB_SPLINE_H_

#include "spline/spline_data.hpp"

namespace interpolation {

#define SPLINE_MAX_ORDER   3
#define SPLINE_CLEARED     {0,0,{0,}}
#define SPLINE_EPS_6       1.0e-6
#define SPLINE_EPS_8       1.0e-8
#define SPLINE_EPS_9       1.0e-9   // 2026-09-10: JointSpline 求值入口边界容差
#define SPLINE_1E6         1.0e6
#define SPLINE_1E8         1.0e8
#define SPLINE_NAN         9.0e9

typedef struct {
   int order;
   double delta_x;
   double c[SPLINE_MAX_ORDER + 1];
} SPLINE;

class CSpline {
private:
    spline::SplineData own_data_;
    spline::SplineData* data_;

public:
    CSpline();
    explicit CSpline(spline::SplineData* external_data);
    CSpline(const CSpline& other);
    CSpline& operator=(const CSpline& other);
    ~CSpline();

    void AttachTo(spline::SplineData* external_data);
    void Detach();

    bool create_complete(const double in_delta_x,
        const double y_1,
        const double y_2,
        const double dy1,
        const double dy2);

    bool init(const int in_order,
        const double in_delta_x,
        const double * in_c);

    bool eval(const double x, double * y) const;
    bool eval_derivative(const double x, double * dy) const;
    bool eval_second_derivative(const double x, double * d2y) const;
    bool eval_third_derivative(const double x, double * d3y) const;

    void clear();
    bool is_empty() const;
    double get_length() const;
    int get_order();

    void spline_get_coeff_from_derivative(double *dc0, double *dc1, double *dc2);
    void spline_get_coeff_from_second_derivative(double *d2c0, double *d2c1);
    void spline_UPROBE(const SPLINE * p);

    spline::SplineData* GetData() { return data_; }
    const spline::SplineData* GetData() const { return data_; }
};

} // namespace interpolation

#endif
