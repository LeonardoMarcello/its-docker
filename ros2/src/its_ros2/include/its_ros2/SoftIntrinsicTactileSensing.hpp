#ifndef SOFT_ITS_H
#define SOFT_ITS_H

#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <memory>
#include <map>
#include <Eigen/Dense>
#include <Eigen/LU>

#include "its_ros2/Fingertip.hpp"
#include "its_ros2/IntrinsicTactileSensing.hpp"

using namespace fingertip;

namespace soft_its {

// ============================================================================
//  Contact Sensing problem minimal solution  (adds deformation Dd)
// ============================================================================
struct ContactSensingProblemSolution {
    Eigen::Vector3d c  {Eigen::Vector3d::Zero()};  // Point of Contact [mm]
    double          K  {0.0};                      // Torque scale factor
    double          Dd {0.0};                      // Surface deformation [mm]
};

// ============================================================================
//  Extended solution  (adds Dd)
// ============================================================================
struct ExtendedContactSensingProblemSolution {
    Eigen::Vector3d PoC {Eigen::Vector3d::Zero()};  // PoC in {B} [mm]
    Eigen::Vector3d n   {Eigen::Vector3d::Zero()};  // Outward normal at PoC
    double          fn  {0.0};                      // Normal force [N]
    Eigen::Vector3d ft  {Eigen::Vector3d::Zero()};  // Tangential force [N]
    double          t   {0.0};                      // Torque along normal [Nmm]
    double          Dd  {0.0};                      // Surface deformation [mm]
};

// ============================================================================
//  SoftIntrinsicTactileSensing
// ============================================================================
class SoftIntrinsicTactileSensing : public its::IntrinsicTactileSensing {
public:
    // Soft-ITS solution (shadows base-class X intentionally)
    ContactSensingProblemSolution X;

    SoftIntrinsicTactileSensing();
    ~SoftIntrinsicTactileSensing() override;

    // ---- fingertip stiffness -----------------------------------------------
    bool setFingertipStiffness(double a = 0.0, double b = 0.0);
    /*  a=0, b=0  →  Rigid
     *  b=0       →  Hooke:     F = a·Δx
     *  both ≠ 0  →  Quadratic: F = a·Δx + b·Δx²
     */

    // ---- solver ------------------------------------------------------------
    int solveContactSensingProblem(
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0,
            its::ContactSensingProblemMethod method =
                its::ContactSensingProblemMethod::Levenberg_Marquardt,
            ContactSensingProblemSolution X0 = {},
            int count_max = 100, double stop_th = 0.005,
            double epsilon = 0.1, bool verbose = false);

    ExtendedContactSensingProblemSolution getExtendedSolution();

    // ---- optimised LM via levmar -------------------------------------------
    int  solveContactSensingProblemOptim(ContactSensingProblemSolution X0,
                                         Eigen::Vector3d f,
                                         Eigen::Vector3d m,
                                         its::OptimLM params);
    void setLMParameters(double forceThreshold = 0.0,
                         int count_max = 100,
                         double stop_th  = 0.005,
                         double epsilon  = 0.005,
                         bool verbose    = false) override;

protected:
    int solveContactSensingProblemLM(
            ContactSensingProblemSolution X0,
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0,
            int count_max = 100, double stop_th = 0.005,
            double epsilon = 0.1, bool verbose = false);

    int solveContactSensingProblemGN(
            ContactSensingProblemSolution X0,
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0,
            int count_max = 100, double stop_th = 0.005,
            double epsilon = 0.005, bool verbose = false);

    int solveContactSensingProblemCF(
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0);

    int solveContactSensingProblemWM(
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0);

private:
    static void soft_contact_sensing_problem(
            double* x, double* g, int m, int n, void* data);
    static void jacobian_soft_contact_sensing_problem(
            double* x, double* jac, int m, int n, void* data);

    // Mesh path: numerical Jacobian via finite differences (dlevmar_dif)
    static void soft_contact_sensing_problem_mesh(
            double* x, double* g, int m, int n, void* data);
};

}  // namespace soft_its
#endif  // SOFT_ITS_H
