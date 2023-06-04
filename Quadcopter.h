#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include <eigen3/Eigen/Dense>

class Quadcopter {
public:
    Quadcopter();
    void setInitialState(const Eigen::VectorXd& state);
    void setControlInputs(const Eigen::VectorXd& controls);
    void updateState(double dt);
    const Eigen::VectorXd& getState() const;

private:
    Eigen::VectorXd state_;     // 12 states [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]
    Eigen::VectorXd controls_;  // 4 control inputs [thrust, roll, pitch, yaw]

    // Quadcopter parameters
    double mass_;
    double g_;
    double L_;
    // ...
};

#endif // QUADCOPTER_H
