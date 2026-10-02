#ifndef ITS_H
#define ITS_H

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
#include "its_ros2/levmar.h"

using namespace fingertip;

namespace its {


// ============================================================================
//  Contact Sensing problem minimal solution (Soft Finger Contact Model)
// ============================================================================
struct ContactSensingProblemSolution {
    Eigen::Vector3d c {Eigen::Vector3d::Zero()};  // Point of Contact [mm]
    double K {0.0};                               // Torque scale factor
};
// ============================================================================
//  Contact Sensing problem extended solution (Soft Finger Contact Model)
// ============================================================================
struct ExtendedContactSensingProblemSolution {
    Eigen::Vector3d PoC {Eigen::Vector3d::Zero()};  // Point of Contact w.r.t {B} [mm]
    Eigen::Vector3d n   {Eigen::Vector3d::Zero()};  // Outward normal at PoC
    double          fn  {0.0};                      // Normal force [N]
    Eigen::Vector3d ft  {Eigen::Vector3d::Zero()};  // Tangential force [N]
    double          t   {0.0};                      // Torque along normal [Nmm]
};



// ============================================================================
//  Contact Sensing problem solver methods
// ============================================================================
enum class ContactSensingProblemMethod : unsigned short int {
    Levenberg_Marquardt = 1,
    Gauss_Newton        = 2,
    Closed_Form         = 3,
    Wrench_Method       = 4,
    Custom              = 5
};
// ============================================================================
//  Parameters for the levmar optimised LM solver
// ============================================================================
struct OptimLM {
    int    m    {4};               // solution dimension
    int    n    {4};               // system dimension
    int    itmax{100};             // max iterations
    double opt [LM_OPTS_SZ]  {};   // solver thresholds
    double info[LM_INFO_SZ]  {};   // solver result info
    double force_threshold {0.0};
    bool   verbose         {false};
};

// ============================================================================
//  Helper: transform sensor-frame (f,m) → fingertip frame {B} in [N, Nmm]
// ============================================================================
inline void transformToFingertipFrame(const Eigen::Vector3d& f_s,  // [N]
                                      const Eigen::Vector3d& m_s,  // [N mm]
                                      const Eigen::Matrix3d& R_sb, // [-]
                                      const Eigen::Vector3d& d_sb, // [mm]
                                      Eigen::Vector3d& f_b,
                                      Eigen::Vector3d& m_b) {
    f_b = R_sb * f_s;                      // [N]
    m_b = R_sb * (m_s + f_s.cross(d_sb));  // [N mm]
}


// ============================================================================
//  IntrinsicTactileSensing
// ============================================================================
class IntrinsicTactileSensing {
public:
    std::string             sensor_id;
    Fingertip               fingertip;
    Eigen::Vector3d         f {Eigen::Vector3d::Zero()};   // Force [N] in {B}
    Eigen::Vector3d         m {Eigen::Vector3d::Zero()};   // Torque [Nmm] in {B}
    ContactSensingProblemSolution X;                       // Current solution
    OptimLM                 params;                        // LM parameters

    // ---- constructors ------------------------------------------------------
    IntrinsicTactileSensing();
    virtual ~IntrinsicTactileSensing();



    /* ---- fingertip setup ---------------------------------------------------
        * * Arguments:
        * - id:  Fingertip name (string)
        * - a:   Ellipsoid principal x-axis [mm]
        * - b:   Ellipsoid principal y-axis [mm]
        * - c:   Ellipsoid principal z-axis [mm]
    */
    bool setFingertipSurface(const std::string& id,
                             double a = 1.0, double b = 1.0, double c = 1.0);

    /** Load fingertip surface from an OBJ mesh file.
     *  The surface type is set to Mesh; the file is parsed once at startup. */
    bool setFingertipSurfaceMesh(const std::string& id,
                                 const std::string& obj_filepath,
                                 double mesh_scale,
                                 const std::vector<double>& proxy_transform);
    bool setFingertipSurfaceMesh(const std::string& id,
                                 const std::string& obj_filepath,
                                 double mesh_scale = 1.0);

    /** Build fingertip surface as the convex hull
     *  The resulting surface type is set to ConvexHull. */
    bool setFingertipSurfaceConvexHull(const std::string& id,
                                       const std::string& obj_or_xyz_filepath,
                                       double mesh_scale,
                                       const std::vector<double>& proxy_transform);
    bool setFingertipSurfaceConvexHull(const std::string& id,
                                       const std::string& obj_or_xyz_filepath,
                                       double mesh_scale = 1.0);



    bool setFingertipDisplacement(double dispX, double dispY, double dispZ);
    bool setFingertipOrientation(double roll, double pitch, double yaw);



    // ---- solver ------------------------------------------------------------
    /* Dispatcher for InContact Sensing Problem solution
        * * Arguments:
        * - Force:      raw force measurement in sensor frame {S}, [N]
        * - Torque:     raw torque measurement in sensor frame {S}, [N mm]
    */
    int solveContactSensingProblem(
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0,
            ContactSensingProblemMethod method =
                ContactSensingProblemMethod::Levenberg_Marquardt,
            ContactSensingProblemSolution X0 = {},
            int count_max = 100, double stop_th = 0.005,
            double epsilon = 0.1, bool verbose = false);


    int solveContactSensingProblemInitialGuess(Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0);

    ExtendedContactSensingProblemSolution getExtendedSolution();

    // ---- optimized LM via levmar -------------------------------------------
    int  solveContactSensingProblemOptim(ContactSensingProblemSolution X0,
                                         Eigen::Vector3d f, Eigen::Vector3d m,
                                         OptimLM params);
    virtual void setLMParameters(double forceThreshold = 0.0,
                                 int count_max = 100,
                                 double stop_th  = 0.005,
                                 double epsilon  = 0.005,
                                 bool verbose    = false);

protected:
    // Solve with Iterative Levenberg-Marquardt method
    virtual int solveContactSensingProblemLM(
            ContactSensingProblemSolution X0,
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0,
            int count_max = 100, double stop_th = 0.005,
            double epsilon = 0.1, bool verbose = false);

    // Solve with Iterative Gaussian-Newton method
    virtual int solveContactSensingProblemGN(
            ContactSensingProblemSolution X0,
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0,
            int count_max = 100, double stop_th = 0.005,
            double epsilon = 0.005, bool verbose = false);


    // Solve with Closed Form method (Check for Surface Type)
    virtual int solveContactSensingProblemCF(
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0);

    // Solve with Wrench-Method
    virtual int solveContactSensingProblemWM(
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0);


    // Solve with Custom Solver
    virtual int solveContactSensingProblemCUSTOM(
            Eigen::Vector3d f, Eigen::Vector3d t,
            double forceThreshold = 0.0);

private:
    // Static callbacks for levmar library usage
    static void contact_sensing_problem(
            double* x, double* g, int m, int n, void* data);
    static void jacobian_contact_sensing_problem(
            double* x, double* jac, int m, int n, void* data);

    // Static callbacks for levmar – mesh/convex-hull path
    // The surface pointer is passed through the data block.
    static void contact_sensing_problem_mesh(
            double* x, double* g, int m, int n, void* data);
    static void jacobian_contact_sensing_problem_mesh(
            double* x, double* jac, int m, int n, void* data);
};

}  // namespace its
#endif  // ITS_H
