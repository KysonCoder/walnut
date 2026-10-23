#ifndef TH34TRANSFORMS_HPP
#define TH34TRANSFORMS_HPP

#include <cmath>
#include <stdexcept>

//输入输出均为弧度制
class Transforms {
private:
    static constexpr double la = 4.3;
    static constexpr double lb = 5.3;
    static constexpr double l0 = 25.0;
    static constexpr double l1 = 23.155;
    static constexpr double PI = 3.14159265358979323846;

public:
    // 角度转弧度
    static double deg2rad(double deg) {
        return deg * PI / 180.0;
    }

    // 弧度转角度
    static double rad2deg(double rad) {
        return rad * 180.0 / PI;
    }

    // 第一个函数：transf_th3to4
    static double transf_th3to4(double x) {
        double theta_a = PI - x;   // 输入角A (弧度)

        // 计算 θb
        double A = -2.0 * la * lb * std::cos(theta_a);
        double B = -2.0 * la * lb * std::sin(theta_a) + 2.0 * l0 * lb;
        double K = l1 * l1 - la * la - lb * lb - l0 * l0 + 2.0 * l0 * la * std::sin(theta_a);

        double R = std::sqrt(A * A + B * B);
        double phi = std::atan2(B, A);
        double alpha = std::acos(K / R);

        double theta_b = phi - alpha;

        return theta_b + 40.0 * PI /180;
    }

    // 第二个函数：transf_th4to3
    static double transf_th4to3(double x) {
        double target_b = x - 40.0 * PI /180; // 输入角B (弧度)

        // 定义函数 f(a)
        auto f = [target_b](double a) -> double {
            double A_val = -2.0 * la * lb * std::cos(a);
            double B_val = -2.0 * la * lb * std::sin(a) + 2.0 * l0 * lb;
            double numerator = l1 * l1 - la * la - lb * lb - l0 * l0 + 2.0 * l0 * la * std::sin(a);
            double denominator = std::sqrt(A_val * A_val + B_val * B_val);

            double phi = std::atan2(B_val, A_val);
            double alpha = std::acos(numerator / denominator);

            return (phi - alpha) - target_b;
        };

        // 使用二分法求解 f(a) = 0
        double a_low = deg2rad(60.0);   // 搜索区间下限
        double a_high = deg2rad(180.0); // 搜索区间上限
        double a_mid;

        const int max_iter = 100;
        const double tolerance = 1e-10;

        double f_low = f(a_low);
        double f_high = f(a_high);

        // 检查区间端点符号
        if (std::fabs(f_low) < tolerance) {
            a_mid = a_low;
        } else if (std::fabs(f_high) < tolerance) {
            a_mid = a_high;
        } else if (f_low * f_high > 0) {
            return PI;
        } else {
            // 二分法迭代
            for (int i = 0; i < max_iter; ++i) {
                a_mid = (a_low + a_high) / 2.0;
                double f_mid = f(a_mid);

                if (std::fabs(f_mid) < tolerance || (a_high - a_low) / 2.0 < tolerance) {
                    break;
                }

                if (f_mid * f_low > 0) {
                    a_low = a_mid;
                    f_low = f_mid;
                } else {
                    a_high = a_mid;
                }
            }
        }

        double theta_a = PI - a_mid;
        return theta_a;
    }
};

#endif // TH34TRANSFORMS_HPP
