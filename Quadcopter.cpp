#include "Quadcopter.h"
#include <cmath>

Quadcopter::Quadcopter()
{
    // Initialize quadcopter parameters
    mass_ = 1.0;
    g_ = 9.81;
    L_ = 0.25;

    // Initialize state and control inputs
    state_.resize(12);
    controls_.resize(4);
}

void Quadcopter::setInitialState(const Eigen::VectorXd& state)
{
    state_ = state;
}

void Quadcopter::setControlInputs(const Eigen::VectorXd& controls)
{
    controls_ = controls;
}

void Quadcopter::updateState(double dt)
{
    // Extract state variables
    double x = state_(0);
    double y = state_(1);
    double z = state_(2);
    double vx = state_(3);
    double vy = state_(4);
    double vz = state_(5);
    double phi = state_(6);
    double theta = state_(7);
    double psi = state_(8);
    double wx = state_(9);
    double wy = state_(10);
    double wz = state_(11);

    // Extract control inputs
    double thrust = controls_(0);
    double roll = controls_(1);
    double pitch = controls_(2);
    double yaw = controls_(3);

    // Compute trigonometric terms
    double cos_phi = std::cos(phi);
    double sin_phi = std::sin(phi);
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double cos_psi = std::cos(psi);
    double sin_psi = std::sin(psi);

    // Compute forces and moments
    double f_x = -thrust * sin_theta;
    double f_y = thrust * sin_phi * cos_theta;
    double f_z = thrust * cos_phi * cos_theta;
    double m_x = L_ * (thrust * cos_theta * sin_phi - 0.1 * vx);
    double m_y = L_ * (thrust * sin_theta * sin_phi - 0.1 * vy);
    double m_z = 0.1 * wz - 0.01 * wx;

    // Compute accelerations
    double a_x = (f_x / mass_);
    double a_y = (f_y / mass_);
    double a_z = (f_z / mass_);
    double alpha_x = (m_x / mass_);
    double alpha_y = (m_y / mass_);
    double alpha_z = (m_z / mass_);

    // Update state
    state_(0) += dt * vx;
    state_(1) += dt * vy;
    state_(2) += dt * vz;
    state_(3) += dt * a_x;
    state_(4) += dt * a_y;
    state_(5) += dt * a_z;
    state_(6) += dt * wx;
    state_(7) += dt * wy;
    state_(8) += dt * wz;
    state_(9) += dt * alpha_x;
    state_(10) += dt * alpha_y;
    state_(11) += dt * alpha_z;
}

const Eigen::VectorXd& Quadcopter::getState() const
{
    return state_;
}
