#include "its_ros2/IntrinsicTactileSensing.hpp"
#include "its_ros2/its_custom_solver.hpp"
#include <stdexcept>
#include <string>
#include <cstring>

using namespace its;
using namespace fingertip;

// ============================================================================
//  Constructor / Destructor
// ============================================================================
IntrinsicTactileSensing::IntrinsicTactileSensing()  { /* nothing */ }
IntrinsicTactileSensing::~IntrinsicTactileSensing() { /* nothing */ }

// ============================================================================
//  Fingertip setup
// ============================================================================
/*
    setFingertipSurface
    set up finger with elliptic surfaces
*/
bool IntrinsicTactileSensing::setFingertipSurface(const std::string& id,
                                                   double a, double b, double c) {
    fingertip.id = id;
    // Retrieve analytical surface
    if (b == -1 && c == -1 ){
        fingertip.model.surfaceType = SurfaceType::Plane;
        fingertip.model.principalAxisCoeff = {a, b, c};
    } else if (c == -1){
        if (a!=b) {
            std::cout << "[ITS] Error: cylinder must have a = b" << std::endl;
            return false;
        }
        fingertip.model.surfaceType = SurfaceType::Cylinder;
        fingertip.model.principalAxisCoeff = {a, b, c};
    } else if (a == b && b == c){
        fingertip.model.surfaceType = SurfaceType::Sphere;
        fingertip.model.principalAxisCoeff = {a, b, c};
    } else{
        fingertip.model.surfaceType = SurfaceType::Ellipsoid;
        fingertip.model.principalAxisCoeff = {a, b, c};
    }
    return true;
}
/*
    setFingertipSurfaceMesh
    set up finger with explicit mesh
*/
bool IntrinsicTactileSensing::setFingertipSurfaceMesh(const std::string& id,
                                                      const std::string& obj_filepath,
                                                      double mesh_scale,
                                                      const std::vector<double>& proxy_transform) {
    fingertip.id = id;
    bool ok = fingertip.model.mesh.load(obj_filepath, mesh_scale);
    if (ok) {
        fingertip.model.surfaceType = SurfaceType::Mesh;
        // parse ellipsoid approximation
        fingertip.model.mesh.ProxyPrincipalAxisCoeff = fingertip.model.principalAxisCoeff;
        fingertip.model.mesh.ProxyDisplacement = {proxy_transform[0], proxy_transform[1], proxy_transform[2]};
        fingertip.model.mesh.ProxyOrientation = {proxy_transform[3], proxy_transform[4], proxy_transform[5]};
    }
    return ok;
}
bool IntrinsicTactileSensing::setFingertipSurfaceMesh(const std::string& id,
                                                      const std::string& obj_filepath,
                                                      double mesh_scale) {
    fingertip.id = id;
    bool ok = fingertip.model.mesh.load(obj_filepath, mesh_scale);
    if (ok) {
        fingertip.model.surfaceType = SurfaceType::Mesh;
        // parse ellipsoid approximation
        fingertip.model.mesh.ProxyPrincipalAxisCoeff = fingertip.model.principalAxisCoeff;
    }
    return ok;
}
/*
    setFingertipSurfaceConvexHull
    set up finger with explicit Convex Hull
*/
bool IntrinsicTactileSensing::setFingertipSurfaceConvexHull(const std::string& id,
                                                            const std::string& filepath,
                                                            double mesh_scale,
                                                            const std::vector<double>& proxy_transform) {
    fingertip.id = id;
    bool ok = fingertip.model.mesh.load(filepath, mesh_scale);
    if (ok) {
        fingertip.model.mesh.buildConvexHull();
        fingertip.model.surfaceType = SurfaceType::ConvexHull;
        // parse ellipsoid approximation
        fingertip.model.mesh.ProxyPrincipalAxisCoeff = fingertip.model.principalAxisCoeff;
        fingertip.model.mesh.ProxyDisplacement = {proxy_transform[0], proxy_transform[1], proxy_transform[2]};
        fingertip.model.mesh.ProxyOrientation = {proxy_transform[3], proxy_transform[4], proxy_transform[5]};
    }
    return ok;
}
bool IntrinsicTactileSensing::setFingertipSurfaceConvexHull(const std::string& id,
                                                            const std::string& filepath,
                                                            double mesh_scale) {
    fingertip.id = id;
    bool ok = fingertip.model.mesh.load(filepath, mesh_scale);
    if (ok) {
        fingertip.model.mesh.buildConvexHull();
        fingertip.model.surfaceType = SurfaceType::ConvexHull;
        // parse ellipsoid approximation
        fingertip.model.mesh.ProxyPrincipalAxisCoeff = fingertip.model.principalAxisCoeff;
    }
    return ok;
}


/*
    setFingertipDisplacement
    set fingertip relative displacement with respect to f/t sensor
*/
bool IntrinsicTactileSensing::setFingertipDisplacement(double dispX, double dispY, double dispZ) {
    fingertip.displacement = {dispX, dispY, dispZ};
    return true;
}
/*
    setFingertipOrientation
    set fingertip relative orientation with respect to f/t sensor
*/
bool IntrinsicTactileSensing::setFingertipOrientation(double roll, double pitch, double yaw) {
    Eigen::Matrix3d Roll, Pitch, Yaw;
    Roll  << 1,         0,          0,
             0,  cos(roll),  sin(roll),
             0, -sin(roll),  cos(roll);

    Pitch << cos(pitch), 0, -sin(pitch),
             0,          1,           0,
             sin(pitch), 0,  cos(pitch);

    Yaw   <<  cos(yaw), sin(yaw), 0,
             -sin(yaw), cos(yaw), 0,
              0,        0,        1;

    fingertip.orientation = Yaw * Pitch * Roll;
    return true;
}


// ============================================================================
//  Solver dispatcher
// ============================================================================
int IntrinsicTactileSensing::solveContactSensingProblem(
        Eigen::Vector3d f, Eigen::Vector3d m, double forceThreshold,
        ContactSensingProblemMethod method,
        ContactSensingProblemSolution X0,
        int count_max, double stop_th, double epsilon, bool verbose) {

    switch (method) {
        case ContactSensingProblemMethod::Levenberg_Marquardt:
            return solveContactSensingProblemOptim(X0, f, m, this->params);
        case ContactSensingProblemMethod::Gauss_Newton:
            return solveContactSensingProblemGN(X0, f, m, forceThreshold, count_max, stop_th, epsilon, verbose);
        case ContactSensingProblemMethod::Closed_Form:
            return solveContactSensingProblemCF(f, m, forceThreshold);
        case ContactSensingProblemMethod::Wrench_Method:
            return solveContactSensingProblemWM(f, m, forceThreshold);
        case ContactSensingProblemMethod::Custom:
            return solveContactSensingProblemCUSTOM(f, m, forceThreshold);
        default:
            return solveContactSensingProblemLM(X0, f, m, forceThreshold, count_max, stop_th, epsilon, verbose);
    }
}

int IntrinsicTactileSensing::solveContactSensingProblemInitialGuess(Eigen::Vector3d f, Eigen::Vector3d m, double forceThreshold) {
    bool check_cf = (fingertip.model.surfaceType == fingertip::SurfaceType::Plane) ||
                    (fingertip.model.surfaceType == fingertip::SurfaceType::Sphere) ||
                    (fingertip.model.surfaceType == fingertip::SurfaceType::Cylinder) ||
                    (fingertip.model.surfaceType == fingertip::SurfaceType::Ellipsoid);
    if (check_cf) {
        throw std::invalid_argument(
            "[ITS] Error: Initiall guess unecessary with elliptical shapes"
        );
    }

    // 1. store mesh position, orientation, and type (Sensor -> Mesh transform)
    auto mesh_displacement = fingertip.displacement; // Eigen::Vector3d
    auto mesh_orientation = fingertip.orientation;   // Eigen::Matrix3d
    auto surface_type      = fingertip.model.surfaceType;

    // 2. Extract proxy local transform (Mesh -> Proxy transform)
    double r = fingertip.model.mesh.ProxyOrientation[0];
    double p = fingertip.model.mesh.ProxyOrientation[1];
    double y = fingertip.model.mesh.ProxyOrientation[2];

    // Build local Rotation matrix matching your orientation function
    Eigen::Matrix3d Roll, Pitch, Yaw;
    Roll  << 1, 0, 0,  0, cos(r), sin(r),  0, -sin(r), cos(r);
    Pitch << cos(p), 0, -sin(p),  0, 1, 0,  sin(p), 0, cos(p);
    Yaw   << cos(y), sin(y), 0,  -sin(y), cos(y), 0,  0, 0, 1;
    Eigen::Matrix3d R_bp = Yaw * Pitch * Roll; // R_base_to_proxy

    // Build local Translation vector
    Eigen::Vector3d t_bp(
        fingertip.model.mesh.ProxyDisplacement[0],
        fingertip.model.mesh.ProxyDisplacement[1],
        fingertip.model.mesh.ProxyDisplacement[2]
    );

    // 3. Compute proxy position and orientation w.r.t Sensor (T_sp = T_sb * T_bp)
    Eigen::Vector3d t_sp = (mesh_orientation * t_bp) + mesh_displacement;
    Eigen::Matrix3d R_sp = mesh_orientation * R_bp;
    fingertip.model.surfaceType = fingertip::SurfaceType::Ellipsoid;

    // 4. assign proxy transform w.r.t sensor frame directly
    // (Bypassing the setter functions so we don't have to extract Euler angles from R_sp)
    fingertip.displacement = t_sp;
    fingertip.orientation = R_sp;

    // 5. solve initial guess (Solution will be output natively in the Proxy Frame)
    int res = solveContactSensingProblemCF(f, m, forceThreshold);

    // 6. Convert point from Proxy Frame back to Mesh Frame (P_mesh = R_bp * P_proxy + t_bp)
    X.c = (R_bp * X.c) + t_bp;

    // 7. recover mesh position, orientation, and type
    fingertip.displacement = mesh_displacement;
    fingertip.orientation = mesh_orientation;
    fingertip.model.surfaceType = surface_type;

    return res;


}


// ============================================================================
//  Iterative Methods
// ============================================================================

//  Levenberg-Marquardt  (custom implementation)
int IntrinsicTactileSensing::solveContactSensingProblemLM(
        ContactSensingProblemSolution X0, Eigen::Vector3d f_in, Eigen::Vector3d m_in,
        double forceThreshold, int count_max, double stop_th, double epsilon, bool verbose) {

    this->f = f_in; this->m = m_in;
    if (f_in.norm() < forceThreshold) return -1;

    const double a = fingertip.model.principalAxisCoeff[0];
    const double b = fingertip.model.principalAxisCoeff[1];
    const double c = fingertip.model.principalAxisCoeff[2];

    Eigen::Vector3d fb, mb;
    transformToFingertipFrame(f_in, m_in, fingertip.orientation, fingertip.displacement, fb, mb);
    this->f = fb; this->m = mb;

    const double fx = fb(0), fy = fb(1), fz = fb(2);
    const double mx = mb(0), my = mb(1), mz = mb(2);

    this->X = X0;
    double x = X.c(0), y = X.c(1), z = X.c(2), k = X.K;
    double lambda = 100.0;

    Eigen::VectorXd g(4);
    Eigen::MatrixXd J(4, 4);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd h(4);

    auto evalG = [&]() {
        g(0) = 2*k*x/(a*a) - fy*z + fz*y - mx;
        g(1) = 2*k*y/(b*b) - fz*x + fx*z - my;
        g(2) = 2*k*z/(c*c) - fx*y + fy*x - mz;
        g(3) = x*x/(a*a) + y*y/(b*b) + z*z/(c*c) - 1.0;
    };
    auto evalJ = [&]() {
        J(0,0) = 2*k/(a*a); J(0,1) = fz;         J(0,2) = -fy;        J(0,3) = 2*x/(a*a);
        J(1,0) = -fz;        J(1,1) = 2*k/(b*b);  J(1,2) = fx;         J(1,3) = 2*y/(b*b);
        J(2,0) = fy;         J(2,1) = -fx;         J(2,2) = 2*k/(c*c); J(2,3) = 2*z/(c*c);
        J(3,0) = 2*x/(a*a); J(3,1) = 2*y/(b*b);  J(3,2) = 2*z/(c*c); J(3,3) = 0.0;
    };

    evalG();
    double Xi = g.squaredNorm(), Xi_old = Xi;
    int count = 0;

    while (Xi >= stop_th && count <= count_max) {
        evalJ();
        h = -(J.transpose() * J + lambda * I).inverse() * J.transpose() * g;
        x += h(0); y += h(1); z += h(2); k += h(3);
        evalG();

        if (verbose) {
            const Eigen::IOFormat fmt(5, Eigen::DontAlignCols, "\t|", " ");
            std::cout << "Step " << count << "  g=" << g.transpose().format(fmt)
                      << "  x=[" << x << "," << y << "," << z << "," << k << "]\n";
        }

        Xi_old = Xi;
        Xi = g.squaredNorm();
        if ((Xi_old - Xi) > epsilon * h.transpose() * (lambda * h - J.transpose() * g))
            lambda /= 10.0;
        else
            lambda *= 10.0;
        ++count;
    }

    X.c = {x, y, z}; X.K = k;
    return count;
}

//  Gauss-Newton

int IntrinsicTactileSensing::solveContactSensingProblemGN(
        ContactSensingProblemSolution X0, Eigen::Vector3d f_in, Eigen::Vector3d m_in,
        double forceThreshold, int count_max, double stop_th, double epsilon, bool verbose) {

    this->f = f_in; this->m = m_in;
    if (f_in.norm() < forceThreshold) return -1;

    Eigen::Vector3d fb, mb;
    transformToFingertipFrame(f_in, m_in, fingertip.orientation, fingertip.displacement, fb, mb);
    this->f = fb; this->m = mb;

    this->X = X0;

    // State vector array for callbacks: x = [c_x, c_y, c_z, K]
    double x_arr[4] = {X.c(0), X.c(1), X.c(2), X.K};

    const bool is_mesh = (fingertip.model.surfaceType == SurfaceType::Mesh ||
                          fingertip.model.surfaceType == SurfaceType::ConvexHull);

    // Prepare the data blocks required by the callbacks
    double data_analytic[9];
    double data_mesh[8];

    if (is_mesh) {
        data_mesh[0] = fb(0); data_mesh[1] = fb(1); data_mesh[2] = fb(2);
        data_mesh[3] = mb(0); data_mesh[4] = mb(1); data_mesh[5] = mb(2);
        const Surface* surf_ptr = &fingertip.model;
        std::memcpy(&data_mesh[6], &surf_ptr, sizeof(Surface*));

        // Optional: Project initial guess onto the mesh to prevent wild first steps
        Eigen::Vector3d c0 = fingertip.model.projectOnSurface(Eigen::Vector3d(x_arr[0], x_arr[1], x_arr[2]));
        x_arr[0] = c0(0); x_arr[1] = c0(1); x_arr[2] = c0(2);
    } else {
        data_analytic[0] = fb(0); data_analytic[1] = fb(1); data_analytic[2] = fb(2);
        data_analytic[3] = mb(0); data_analytic[4] = mb(1); data_analytic[5] = mb(2);
        data_analytic[6] = fingertip.model.principalAxisCoeff[0];
        data_analytic[7] = fingertip.model.principalAxisCoeff[1];
        data_analytic[8] = fingertip.model.principalAxisCoeff[2];
    }

    // Output arrays for callbacks
    double g_arr[4];
    double jac_arr[16];

    // Eigen wrappers map directly to the memory of the raw arrays.
    // Note: levmar Jacobians are populated row-major, so we explicitly define RowMajor.
    Eigen::Map<Eigen::VectorXd> g(g_arr, 4);
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> J(jac_arr);

    // Helper lambda to call the appropriate callback and update g_arr / jac_arr
    auto evalG_and_J = [&]() {
        if (is_mesh) {
            // use mesh S(*)
            contact_sensing_problem_mesh(x_arr, g_arr, 4, 4, data_mesh);
            jacobian_contact_sensing_problem_mesh(x_arr, jac_arr, 4, 4, data_mesh);
        } else {
            // use elliptical
            contact_sensing_problem(x_arr, g_arr, 4, 4, data_analytic);
            jacobian_contact_sensing_problem(x_arr, jac_arr, 4, 4, data_analytic);
        }
    };

    // Initialize first iteration
    evalG_and_J();
    double Xi = g.squaredNorm();
    int count = 0;

    while (Xi >= stop_th && count <= count_max) {

        // Gradient Descent step
        // h = -epsilon * J^T * g
        Eigen::VectorXd h = -epsilon * J.transpose() * g;

        // Update state
        x_arr[0] += h(0);
        x_arr[1] += h(1);
        x_arr[2] += h(2);
        x_arr[3] += h(3);

        // Re-evaluate
        evalG_and_J();
        Xi = g.squaredNorm();

        if (verbose) {
            const Eigen::IOFormat fmt(5, Eigen::DontAlignCols, "\t|", " ");
            std::cout << "GD Step " << count
                      << "  g = " << g.transpose().format(fmt)
                      << "  ||e||=" << Xi << "\n";
        }

        ++count;
    }

    // Store final converged result
    X.c = {x_arr[0], x_arr[1], x_arr[2]};
    X.K = x_arr[3];
    return count;
}

//  Optimised Levenberg-Marquardt (via levmar.h library)

int IntrinsicTactileSensing::solveContactSensingProblemOptim(
        ContactSensingProblemSolution X0,
        Eigen::Vector3d f_in, Eigen::Vector3d m_in,
        OptimLM p) {

    this->f = f_in; this->m = m_in;
    if (f_in.norm() < p.force_threshold) return -1;

    Eigen::Vector3d fb, mb;
    transformToFingertipFrame(f_in, m_in, fingertip.orientation, fingertip.displacement, fb, mb);
    this->f = fb; this->m = mb;

    const bool use_mesh = (fingertip.model.surfaceType == SurfaceType::Mesh ||
                           fingertip.model.surfaceType == SurfaceType::ConvexHull);

    int step = -1;

    if (!use_mesh) {
        // ---- analytic path ------------------------------------------------
        const double a = fingertip.model.principalAxisCoeff[0];
        const double b2= fingertip.model.principalAxisCoeff[1];
        const double c = fingertip.model.principalAxisCoeff[2];

        double x[4] = {X0.c(0), X0.c(1), X0.c(2), X0.K};
        double data[9] = {fb(0), fb(1), fb(2), mb(0), mb(1), mb(2), a, b2, c};

        step = dlevmar_der(contact_sensing_problem,
                           jacobian_contact_sensing_problem,
                           x, nullptr, p.m, p.n, p.itmax,
                           p.opt, p.info, nullptr, nullptr, data);

        X.c = {x[0], x[1], x[2]}; X.K = x[3];
    } else {
        // ---- mesh path: analytical Jacobian (dlevmar_der) ------------------
        //  Initial guess: project X0.c onto the mesh surface
        Eigen::Vector3d c0 = X0.c;
        if (c0.isZero(1e-9))
            c0 = fingertip.model.mesh.triangles[0].centroid();
        c0 = fingertip.model.projectOnSurface(c0);

        double x[4] = {c0(0), c0(1), c0(2), X0.K};

        // Pack: [fx,fy,fz, mx,my,mz, surface_ptr_in_2_doubles]
        const Surface* surf_ptr = &fingertip.model;
        double data[8] = {fb(0), fb(1), fb(2), mb(0), mb(1), mb(2), 0.0, 0.0};
        std::memcpy(&data[6], &surf_ptr, sizeof(Surface*));

        step = dlevmar_der(contact_sensing_problem_mesh,
                           jacobian_contact_sensing_problem_mesh,
                           x, nullptr, p.m, p.n, p.itmax,
                           p.opt, p.info, nullptr, nullptr, data);

        X.c = {x[0], x[1], x[2]}; X.K = x[3];
    }

    if (p.verbose) {
        std::cout << "[ITS] Levenberg-Marquardt solver Table \n"
                  << "      - levmar stop reason : " << p.info[6]   << "\n"
                  << "      - iterations         : " << p.info[5]   << "\n"
                  << "      - ||e||_2 initial    : " << p.info[0]   << "\n"
                  << "      - ||e||_2 final      : " << p.info[1]   << "\n"
                  << "      - ||J^T e||_inf      : " << p.info[2]   << "\n"
                  << "      - ||Δx||_2           : " << p.info[3]   << "\n"
                  << "      - lambda final       : " << p.info[4]   << "\n";
    }
    return step;
}


// ============================================================================
//  Closed-Form Method
// ============================================================================
int IntrinsicTactileSensing::solveContactSensingProblemCF(
        Eigen::Vector3d f_in, Eigen::Vector3d m_in, double forceThreshold) {


    // Check wheter Closed Form mathod is applicable
    bool check_cf = (fingertip.model.surfaceType == fingertip::SurfaceType::Plane) ||
                    (fingertip.model.surfaceType == fingertip::SurfaceType::Sphere) ||
                    (fingertip.model.surfaceType == fingertip::SurfaceType::Cylinder) ||
                    (fingertip.model.surfaceType == fingertip::SurfaceType::Ellipsoid);
    if (!check_cf) {
        throw std::invalid_argument(
            "[ITS] Error: Closed-Form solver is not applicable for discrete surfaces. "
            "Please select Levenberg-Marquardt or Gauss-Newton for Mesh or ConvexHull types."
        );
    }

    this->f = f_in; this->m = m_in;
    if (f_in.norm() < forceThreshold) return -1;
    // Translate raw measurements in Fingertip frame {B}
    Eigen::Vector3d p, t;
    transformToFingertipFrame(f_in, m_in, fingertip.orientation, fingertip.displacement, p, t);
    this->f = p; this->m = t;


    if (fingertip.model.surfaceType == fingertip::SurfaceType::Sphere){
        double R = fingertip.model.principalAxisCoeff[0]; // a = b = c = Sphere radius
        const double pdott = p.dot(t);
        const double sigma = t.squaredNorm() - R*R*p.squaredNorm();

        const double K = (-(pdott / std::abs(pdott)) / (std::sqrt(2.0) * R))
                    * std::sqrt(sigma + std::sqrt(sigma*sigma + 4.0*R*R*pdott*pdott));
        if (K == 0.0)
            return solveContactSensingProblemWM(f_in, m_in, forceThreshold);  // wrench-axis fallback
        Eigen::Vector3d PoC = 1.0 / (K*(K*K + p.squaredNorm())) *
                            (K*K* t + K * p.cross(t) + pdott * p);

        X.c = PoC;
        X.K = K;

    }else if (fingertip.model.surfaceType == fingertip::SurfaceType::Cylinder){
        double R = fingertip.model.principalAxisCoeff[0]; // a = b = cilinder radius; c = -1
        const double pdott = p.dot(t);

        Eigen::Vector3d pnorm(p(0), p(1), 0.0);
        Eigen::Vector3d ttan(0.0, 0.0, t(2));
        const double pnorm_sqnorm = pnorm.squaredNorm();
        const double ttan_sqnorm  = ttan.squaredNorm();

        const double K = - pdott / std::sqrt(R*R*pnorm_sqnorm - ttan_sqnorm);
        if (K == 0.0)
            return solveContactSensingProblemWM(f_in, m_in, forceThreshold);  // wrench-axis fallback
        Eigen::Vector3d PoC = 1.0 / (K*pnorm_sqnorm) *
                            (K*K* ttan + K * pnorm.cross(t) + pdott * p);

        X.c = PoC;
        X.K = K;

    }else if (fingertip.model.surfaceType == fingertip::SurfaceType::Plane){
        double R = fingertip.model.principalAxisCoeff[0]; // a = Plane XY distance; b = c = -1
        const double pdott = p.dot(t);

        Eigen::Vector3d ptan(0.0, 0.0, p(2));
        const double ptan_norm    = ptan.norm();
        const double ptan_sqnorm  = ptan.squaredNorm();

        const double K = - pdott /(R*ptan_norm);
        Eigen::Vector3d PoC = 1.0 / (ptan_sqnorm) *
                            (ptan.cross(t) + R*ptan_norm * p);

        X.c = PoC;
        X.K = K;

    }else{
        // General ellipsoid case
        const double a = fingertip.model.principalAxisCoeff[0];
        const double b = fingertip.model.principalAxisCoeff[1];
        const double c = fingertip.model.principalAxisCoeff[2];

        // scale axes to integers for numerical stability
        double alpha = a, beta = b, gamma = c, invR = 1.0;
        while (alpha < 1.0 || beta < 1.0 || gamma < 1.0) {
            invR  *= 10.0;
            alpha  = a * invR;
            beta   = b * invR;
            gamma  = c * invR;
        }
        const double R = 1.0 / invR;

        Eigen::Matrix3d A;
        A << 1.0/alpha, 0,        0,
            0,        1.0/beta,  0,
            0,        0,         1.0/gamma;

        const Eigen::Vector3d Ap    = A * p;
        const Eigen::Vector3d AAp   = A * A * p;
        const Eigen::Vector3d invAt = A.inverse() * t;
        const double D     = A.determinant();
        const double sigma = D*D * invAt.squaredNorm() - R*R * Ap.squaredNorm();
        const double pdott = p.dot(t);

        const double K = (-(pdott / std::abs(pdott)) / (std::sqrt(2.0) * R * D))
                    * std::sqrt(sigma + std::sqrt(sigma*sigma + 4.0*D*D*R*R*pdott*pdott));
        const double detG = K * (K*K*D*D + Ap.squaredNorm());

        if (K == 0.0)
            return solveContactSensingProblemWM(f_in, m_in, forceThreshold);  // wrench-axis fallback

        Eigen::Vector3d PoC = (1.0 / detG) *
                            (K*K*D*D * A.inverse() * A.inverse() * t
                            + K * AAp.cross(t)
                            + pdott * p);

        X.c = PoC;
        X.K = K;
    }


    return 1;
}

// ============================================================================
//  Wrench Method  (Bicchi, Salisbury & Brock, IJRR 1993)
//
//  The measured wrench (f, m) defines the wrench (central) axis
//      c(λ) = r0 + λ·u,   r0 = (f × m) / |f|²,   u = f / |f|
//  For a point contact (m = c × f) the PoC lies on this axis, hence it is the
//  intersection of the axis with the fingertip surface S(c) = 0. Of the two
//  intersections the one where the force pushes into the surface
//  (f·∇S < 0, i.e. the entry point along u) is selected.
//  The torsional moment is then recovered from the residual torque:
//      K = (m - c × f)·∇S / |∇S|²
//  If the axis misses the surface, the axis point closest to the fingertip is
//  projected onto S.
// ============================================================================
int IntrinsicTactileSensing::solveContactSensingProblemWM(
        Eigen::Vector3d f_in, Eigen::Vector3d m_in, double forceThreshold) {

    this->f = f_in; this->m = m_in;
    if (f_in.norm() < forceThreshold || f_in.norm() < 1e-12) return -1;
    // Translate raw measurements in Fingertip frame {B}
    Eigen::Vector3d p, t;
    transformToFingertipFrame(f_in, m_in, fingertip.orientation, fingertip.displacement, p, t);
    this->f = p; this->m = t;

    // Wrench axis
    const Eigen::Vector3d r0 = p.cross(t) / p.squaredNorm();
    const Eigen::Vector3d u  = p.normalized();

    const SurfaceType type = fingertip.model.surfaceType;
    const double a = fingertip.model.principalAxisCoeff[0];
    const double b = fingertip.model.principalAxisCoeff[1];
    const double c = fingertip.model.principalAxisCoeff[2];

    // Gradient ∇S(c) consistent with the residuals of each surface type
    auto gradS = [&](const Eigen::Vector3d& q) -> Eigen::Vector3d {
        switch (type) {
            case SurfaceType::Plane:    return {0.0, 0.0, 1.0};
            case SurfaceType::Cylinder: return {2.0*q(0)/(a*a), 2.0*q(1)/(a*a), 0.0};
            default:                    return fingertip.model.getNormal(q(0), q(1), q(2));
        }
    };

    Eigen::Vector3d PoC;
    bool hit = false;

    if (type == SurfaceType::Mesh || type == SurfaceType::ConvexHull) {
        // ---- mesh path: line-triangle intersection (Möller–Trumbore) -------
        double lambda_best = std::numeric_limits<double>::max();
        for (const auto& tri : fingertip.model.mesh.triangles) {
            if (tri.normal().dot(u) >= 0.0) continue;       // keep entry faces only
            const Eigen::Vector3d e1 = tri.v1 - tri.v0;
            const Eigen::Vector3d e2 = tri.v2 - tri.v0;
            const Eigen::Vector3d h  = u.cross(e2);
            const double det = e1.dot(h);
            if (std::abs(det) < 1e-12) continue;
            const Eigen::Vector3d s = r0 - tri.v0;
            const double bu = s.dot(h) / det;
            if (bu < 0.0 || bu > 1.0) continue;
            const Eigen::Vector3d q = s.cross(e1);
            const double bv = u.dot(q) / det;
            if (bv < 0.0 || bu + bv > 1.0) continue;
            const double lambda = e2.dot(q) / det;
            if (lambda < lambda_best) { lambda_best = lambda; hit = true; }
        }
        if (hit) {
            PoC = r0 + lambda_best * u;
        } else {
            // axis point closest to the mesh centroid, snapped on the surface
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const auto& tri : fingertip.model.mesh.triangles) centroid += tri.centroid();
            centroid /= static_cast<double>(fingertip.model.mesh.triangles.size());
            PoC = fingertip.model.projectOnSurface(r0 + (centroid - r0).dot(u) * u);
        }

    } else if (type == SurfaceType::Plane) {
        // ---- plane z = a ---------------------------------------------------
        if (std::abs(u(2)) < 1e-12) return -1;              // axis parallel to plane
        PoC = r0 + ((a - r0(2)) / u(2)) * u;
        hit = true;

    } else {
        // ---- quadric path: (r0+λu)ᵀ A (r0+λu) = 1 --------------------------
        //  Sphere/Ellipsoid: A = diag(1/a², 1/b², 1/c²)
        //  Cylinder:         A = diag(1/a², 1/a², 0)
        const Eigen::Vector3d Ad = (type == SurfaceType::Cylinder)
            ? Eigen::Vector3d(1.0/(a*a), 1.0/(a*a), 0.0)
            : Eigen::Vector3d(1.0/(a*a), 1.0/(b*b), 1.0/(c*c));

        const double qa = u.cwiseProduct(Ad).dot(u);
        const double qb = u.cwiseProduct(Ad).dot(r0);
        const double qc = r0.cwiseProduct(Ad).dot(r0) - 1.0;
        if (qa < 1e-20) return -1;                           // axis parallel to cylinder

        const double disc = qb*qb - qa*qc;
        if (disc >= 0.0) {
            // smaller root = entry point along u, where f·∇S < 0
            PoC = r0 + ((-qb - std::sqrt(disc)) / qa) * u;
            hit = true;
        } else {
            // axis point closest to the surface (in the A-metric), projected on S
            PoC = r0 - (qb / qa) * u;
            if (type == SurfaceType::Cylinder) {
                const double rho = std::hypot(PoC(0), PoC(1));
                if (rho > 1e-12) { PoC(0) *= a / rho; PoC(1) *= a / rho; }
            } else {
                PoC = fingertip.model.projectOnSurface(PoC);
            }
        }
    }

    // Torsional moment from residual torque along the normal
    const Eigen::Vector3d gS = gradS(PoC);
    const double gS_sq = gS.squaredNorm();
    X.c = PoC;
    X.K = (gS_sq > 1e-20) ? (t - PoC.cross(p)).dot(gS) / gS_sq : 0.0;

    return hit ? 1 : 0;
}

// ============================================================================
//  levmar callbacks
//
//  Residual equations:
//    g[0..2] = (K·n̂(c) + c × f) - m  : torque constraint [N mm], 3 eqs
//    g[3]    = S(c) = 0              : point on surface [mm], 1 eq signed-distance
//
//  Jacobian J = ∂g/∂x ∈ R^(4x4):
//    J = [ K·H(c) - [f]_x  |  ∇S(c) ]
//        [     ∇S(c)^T     |    0   ]
//

//  –  analytic path
//  The data block carries: [fx, fy, fz, mx, my, mz, a, b, c]
//  where a, b, c, are the principal axes of the ellipsoidal shape.
void IntrinsicTactileSensing::contact_sensing_problem(
        double* x, double* g, int /*m*/, int /*n*/, void* data) {
    double input[9];
    std::memcpy(input, data, 9 * sizeof(double));
    const double fx=input[0], fy=input[1], fz=input[2];
    const double mx=input[3], my=input[4], mz=input[5];
    const double a=input[6],  b=input[7],  c=input[8];

    // residual equations:
    // (K·n̂(c) + c × f) - m, [N mm]
    g[0] = 2*x[3]*x[0]/(a*a) - fy*x[2] + fz*x[1] - mx;
    g[1] = 2*x[3]*x[1]/(b*b) - fz*x[0] + fx*x[2] - my;
    g[2] = 2*x[3]*x[2]/(c*c) - fx*x[1] + fy*x[0] - mz;
    // S(c), [mm]
    g[3] = x[0]*x[0]/(a*a) + x[1]*x[1]/(b*b) + x[2]*x[2]/(c*c) - 1.0;
}
void IntrinsicTactileSensing::jacobian_contact_sensing_problem(
        double* x, double* jac, int /*m*/, int /*n*/, void* data) {
    double input[9];
    std::memcpy(input, data, 9 * sizeof(double));
    const double fx=input[0], fy=input[1], fz=input[2];
    const double a=input[6],  b=input[7],  c=input[8];

    // jacobian: J = ∂g/∂x ∈ R^(4x4)
    jac[0] = 2*x[3]/(a*a); jac[1] = fz;           jac[2] = -fy;          jac[3] = 2*x[0]/(a*a);
    jac[4] = -fz;           jac[5] = 2*x[3]/(b*b); jac[6] = fx;           jac[7] = 2*x[1]/(b*b);
    jac[8] = fy;            jac[9] = -fx;           jac[10]= 2*x[3]/(c*c); jac[11]= 2*x[2]/(c*c);
    jac[12]= 2*x[0]/(a*a); jac[13]= 2*x[1]/(b*b); jac[14]= 2*x[2]/(c*c); jac[15]= 0.0;
}

// –  mesh path
//  The data block carries: [fx, fy, fz, mx, my, mz, ptr_as_two_doubles]
//  where ptr_as_two_doubles is the Surface* packed into two doubles.
void IntrinsicTactileSensing::contact_sensing_problem_mesh(
        double* x, double* g, int /*m*/, int /*n*/, void* data) {
    double input[8];
    std::memcpy(input, data, 8 * sizeof(double));
    const double fx=input[0], fy=input[1], fz=input[2];
    const double mx=input[3], my=input[4], mz=input[5];
    Eigen::Vector3d f_b{fx, fy, fz};
    Eigen::Vector3d m_b{mx, my, mz};
    // retrieve Surface pointer stored in the last two doubles
    const Surface* surf = nullptr;
    std::memcpy(&surf, &input[6], sizeof(Surface*));

    // residual equations:
    // (K·n̂(c) + c × f) - m, [N mm]
    Eigen::Vector3d n  = surf->getNormal(x[0], x[1], x[2]).normalized();
    g[0] = x[3] * n.x() - fy*x[2] + fz*x[1] - mx;
    g[1] = x[3] * n.y() - fz*x[0] + fx*x[2] - my;
    g[2] = x[3] * n.z() - fx*x[1] + fy*x[0] - mz;

    // S(c), [mm]
    Eigen::Vector3d c{x[0], x[1], x[2]};
    g[3] = surf->evaluate(c);
}
void IntrinsicTactileSensing::jacobian_contact_sensing_problem_mesh(
        double* x, double* jac, int /*m*/, int /*n*/, void* data) {

    double input[8];
    std::memcpy(input, data, 8 * sizeof(double));
    const double fx = input[0], fy = input[1], fz = input[2];

    // Unpack Surface pointer stored in the last two doubles
    const Surface* surf = nullptr;
    std::memcpy(&surf, &input[6], sizeof(Surface*));

    Eigen::Vector3d c{x[0], x[1], x[2]};

    // ∇S(c) = outward normal of the nearest triangle
    Eigen::Vector3d n = surf->getNormal(c.x(), c.y(), c.z()).normalized();

    // K·H(c) - [f]_x                                   |     ∇S(c)
    jac[0]  =  0.0;   jac[1]  =  fz;    jac[2]  = -fy;    jac[3]  =  n(0);
    jac[4]  = -fz;    jac[5]  =  0.0;   jac[6]  =  fx;    jac[7]  =  n(1);
    jac[8]  =  fy;    jac[9]  = -fx;    jac[10] =  0.0;   jac[11] =  n(2);
    // ∇S(c)^T                                          |      0
    jac[12] =  n(0);  jac[13] =  n(1);  jac[14] =  n(2);   jac[15] =  0.0;
}

// ============================================================================
//  Extended solution
// ============================================================================
ExtendedContactSensingProblemSolution IntrinsicTactileSensing::getExtendedSolution() {
    ExtendedContactSensingProblemSolution csps;
    Eigen::Vector3d n = fingertip.model.getNormal(X.c(0), X.c(1), X.c(2)).normalized();
    csps.PoC = X.c;                                                                 // Contact point [mm] w.r.t. fingertip frame {B}
    csps.n   = n;                                                                   // Normal to surface at PoC [unit vector] w.r.t. fingertip frame {B}
    csps.fn  = this->f.dot(n);                                                      // Normal force [N] w.r.t. fingertip frame {B}
    csps.ft  = this->f - csps.fn * n;                                               // Tangential force [N] w.r.t. fingertip frame {B}
    csps.t   = (X.K * fingertip.model.getNormal(X.c(0), X.c(1), X.c(2))).norm();    // Torsional torque [N mm] w.r.t. fingertip frame {B}
    return csps;
}


// ============================================================================
//  Iterative solver parameters
// ============================================================================
void IntrinsicTactileSensing::setLMParameters(double forceThreshold, int count_max,
                                               double stop_th, double /*epsilon*/,
                                               bool verbose) {
    params.force_threshold = forceThreshold;
    params.itmax           = count_max;
    params.m               = 4;
    params.n               = 4;
    params.verbose         = verbose;

    params.opt[0] = 1e-3;       // initial μ scale
    params.opt[1] = 1e-15;      // stop: ||J^T e||_inf
    params.opt[2] = 1e-15;      // stop: ||Δp||_2
    params.opt[3] = stop_th;    // stop: ||e||_2
}
