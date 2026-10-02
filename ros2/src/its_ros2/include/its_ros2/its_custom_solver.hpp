/**
 * @file its_custom_solver.hpp
 * @brief Custom Solver Template for Intrinsic Tactile Sensing (ITS)
 * * ============================================================================
 * USAGE INSTRUCTIONS:
 * ============================================================================
 * This file provides a template to override the default ITS solvers (LM, GN, CF)
 * with your own custom algorithm (e.g., a neural network, custom geometric
 * heuristic, or specialized optimization loop).
 * * To use this solver in your main application:
 * 1. Implement your math in "STEP 2" below.
 * 2. Save the resulting Point of Contact to `this->X.c` and scale to `this->X.K`.
 * 3. Delete the `throw std::invalid_argument` line.
 * 4. Call `solveContactSensingProblem()` and pass:
 * `method = ContactSensingProblemMethod::Custom`
 */

#ifndef ITS_CUSTOM_SOLVER_HPP
#define ITS_CUSTOM_SOLVER_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <memory>
#include <map>
#include <Eigen/Dense>
#include <Eigen/LU>

#include "its_ros2/IntrinsicTactileSensing.hpp"

namespace its {

    /**
     * @brief Custom solver for the Soft Finger Contact Model
     * * @param f Raw force vector [N] from the sensor.
     * @param m Raw torque vector [Nmm] from the sensor.
     * @param forceThreshold Minimum force required to attempt a solution.
     * @return int representing solver iterations, or 1 for success, -1 for failure/skip.
     * * @note This function is marked `inline` to allow safe inclusion across multiple
     * translation units (.cpp files) without violating the One Definition Rule.
     */
    inline int IntrinsicTactileSensing::solveContactSensingProblemCUSTOM(
            Eigen::Vector3d f_in, Eigen::Vector3d m_in,
            double forceThreshold)
    {

        this->f = f_in;
        this->m = m_in;
        // Skip solver if the applied force is just sensor noise
        if (f_in.norm() < forceThreshold) {
            return -1;
        }
        // Translate raw sensor measurements into the local Fingertip frame {B}
        Eigen::Vector3d f_b, m_b;
        transformToFingertipFrame(f_in, m_in, fingertip.orientation, fingertip.displacement, f_b, m_b);
        // Update class members with the transformed frame data
        this->f = f_b;
        this->m = m_b;

        // ====================================================================
        /* * Write your custom contact sensing algorithm here.
         * * Available context you can use:
         * - this->f (Force in fingertip frame)
         * - this->m (Torque in fingertip frame)
         * - fingertip.model.surfaceType (To check if it's a Mesh, Sphere, Plane, etc.)
         * - fingertip.model.principalAxisCoeff (If dealing with analytic shapes)
         *
         * * Output format:
         * - this->X.c = Eigen::Vector3d(calculated_x, calculated_y, calculated_z);
         * - this->X.K = calculated_torque_scale;
         * - return integer
         */

        // TODO: Remove this throw statement once your logic is implemented
        throw std::invalid_argument(
            "\033[1;31m[ITS] Error: Custom method is not implemented. Define it in 'its_custom_solver.hpp'\033[0m"
        );

        return -1;
    }

} // namespace its

#endif // ITS_CUSTOM_SOLVER_HPP
