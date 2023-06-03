#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include <eigen3/Eigen/Dense>

// Quadcopter class
class Quadcopter {
public:
    // Constructor
    Quadcopter();

    // Destructor
    ~Quadcopter();

    // Set control inputs
    void setControlInputs(double u1, double u2, double u3, double u4);

    // Update quadcopter state
    void updateState(double dt);

    // Get current quadcopter state
    Eigen::VectorXd getState();

private:
    // Quadcopter parameters
    const double g;  // acceleration due to gravity (m/s^2)
    const double m;  // mass of the quadcopter (kg)
    const double l;  // length of each arm (m)
    const double k;  // motor constant
    const double b;  // drag constant

    // Quadcopter state
    Eigen::VectorXd state;

    // Helper functions
    Eigen::MatrixXd getRotationMatrix();
    Eigen::VectorXd calculateForces();
    Eigen::VectorXd calculateMoments();
};

#endif
