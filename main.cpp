#include <iostream>
#include <fstream>
#include "Quadcopter.h"
#include "Quadcopter.cpp"

int main()
{
    // Create a quadcopter object
    Quadcopter quadcopter;

    // Set initial state and control inputs
    Eigen::VectorXd initialState(12);
    initialState << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    quadcopter.setInitialState(initialState);

    Eigen::VectorXd controlInputs(4);
    controlInputs << 1.0, 0.0, 0.0, 0.0;
    quadcopter.setControlInputs(controlInputs);

    // Create a text file to store the quadcopter states
    std::ofstream outputFile("quadcopter_states.txt");
    if (!outputFile.is_open()) {
        std::cout << "Error creating the output file!" << std::endl;
        return 1;
    }

    // Simulate quadcopter dynamics for 1 second with a time step of 0.01 seconds
    double dt = 0.01;
    int numSteps = 100;
    for (int i = 0; i < numSteps; ++i) {
        quadcopter.updateState(dt);
        const Eigen::VectorXd& state = quadcopter.getState();

        // Write the state to the output file
        for (int j = 0; j < state.size(); ++j) {
            outputFile << state(j) << " ";
        }
        outputFile << std::endl;
    }

    // Close the output file
    outputFile.close();

    std::cout << "Simulation completed. Quadcopter states saved to 'quadcopter_states.txt'." << std::endl;

    return 0;
}

