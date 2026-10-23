#ifndef MOTOR_MODEL_HPP_
#define MOTOR_MODEL_HPP_

namespace pathplanning {

struct MotorParams {
    double gear_ratio;    // N:1, motor -> joint
    double stall_torque;  // Nm, torque at omega = 0
    double no_load_speed; // rad/s, speed where tau = 0
};

struct GearboxParams {
    double max_torque;  // Nm
    double min_torque;  // Nm (typically -max_torque)
};

class MotorModel {
public:
    explicit MotorModel(const MotorParams& params);

    // Max positive torque at given motor speed (always >= 0)
    double MaxTorque(double motor_speed) const;

    // Min negative torque at given motor speed (always <= 0)
    double MinTorque(double motor_speed) const;

    // Accessor for gear ratio
    double GearRatio() const { return params_.gear_ratio; }

private:
    MotorParams params_;
};

} // namespace pathplanning

#endif
