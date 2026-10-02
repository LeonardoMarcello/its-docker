#include "its_ros2/SoftIntrinsicTactileSensing.hpp"
#include <cstring>
#include <cmath>

using namespace soft_its;
using namespace fingertip;

// ============================================================================
//  Constructor / Destructor
// ============================================================================
SoftIntrinsicTactileSensing::SoftIntrinsicTactileSensing()  { /* nothing */ }
SoftIntrinsicTactileSensing::~SoftIntrinsicTactileSensing() { /* nothing */ }

// ============================================================================
//  Stiffness
// ============================================================================
bool SoftIntrinsicTactileSensing::setFingertipStiffness(double a, double b) {
    if (a == 0.0 && b == 0.0)
        fingertip.model.stiffnessType = StiffnessType::Rigid;
    else if (b == 0.0)
        fingertip.model.stiffnessType = StiffnessType::Hooke;
    else
        fingertip.model.stiffnessType = StiffnessType::Quadratic;

    fingertip.model.stiffnessCoefficients = {a, b};
    return true;
}

// ============================================================================
//  Helper: transform sensor-frame (f,m) → fingertip frame {B}
// ============================================================================
static void transformToFingertipFrame(const Eigen::Vector3d& f_s,
                                      const Eigen::Vector3d& m_s,
                                      const Eigen::Matrix3d& R_sb,
                                      const Eigen::Vector3d& d_sb,
                                      Eigen::Vector3d& f_b,
                                      Eigen::Vector3d& m_b) {
    f_b = R_sb * f_s;
    m_b = R_sb * (m_s + f_s.cross(d_sb / 1000.0)) * 1000.0;  // → [Nmm]
}

// ============================================================================
//  Solve dispatcher
// ============================================================================
int SoftIntrinsicTactileSensing::solveContactSensingProblem(
        Eigen::Vector3d f, Eigen::Vector3d m, double forceThreshold,
        its::ContactSensingProblemMethod method,
        ContactSensingProblemSolution X0,
        int count_max, double stop_th, double epsilon, bool verbose) {

    switch (method) {
        case its::ContactSensingProblemMethod::Levenberg_Marquardt:
            return solveContactSensingProblemOptim(X0, f, m, this->params);
        case its::ContactSensingProblemMethod::Gauss_Newton:
            return solveContactSensingProblemGN(X0, f, m, forceThreshold, count_max, stop_th, epsilon, verbose);
        case its::ContactSensingProblemMethod::Closed_Form:
            return solveContactSensingProblemCF(f, m, forceThreshold);
        default:
            return solveContactSensingProblemLM(X0, f, m, forceThreshold, count_max, stop_th, epsilon, verbose);
    }
}

// ============================================================================
//  Levenberg-Marquardt  (hand-coded, analytic surface)
// ============================================================================
int SoftIntrinsicTactileSensing::solveContactSensingProblemLM(
        ContactSensingProblemSolution X0, Eigen::Vector3d f_in, Eigen::Vector3d m_in,
        double forceThreshold, int count_max, double stop_th, double epsilon, bool verbose) {

    this->f = f_in; this->m = m_in;
    if (f_in.norm() < forceThreshold) return -1;

    const double a = fingertip.model.principalAxisCoeff[0];
    const double b = fingertip.model.principalAxisCoeff[1];
    const double c = fingertip.model.principalAxisCoeff[2];
    const std::vector<double>& E = fingertip.model.stiffnessCoefficients;

    Eigen::Vector3d fb, mb;
    transformToFingertipFrame(f_in, m_in, fingertip.orientation, fingertip.displacement, fb, mb);
    this->f = fb; this->m = mb;

    const double fx = fb(0), fy = fb(1), fz = fb(2);
    const double mx = mb(0), my = mb(1), mz = mb(2);

    this->X = X0;
    double x = X.c(0), y = X.c(1), z = X.c(2), k = X.K, d = X.Dd;
    double lambda = 100.0;

    Eigen::VectorXd g(5);
    Eigen::MatrixXd J(5, 5);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
    Eigen::VectorXd h(5);

    auto evalG = [&]() {
        Eigen::Vector3d n = fingertip.model.getNormal(x, y, z, d).normalized();
        g(0) = 2*k*x/((a-d)*(a-d)) - fy*z + fz*y - mx;
        g(1) = 2*k*y/((b-d)*(b-d)) - fz*x + fx*z - my;
        g(2) = 2*k*z/((c-d)*(c-d)) - fx*y + fy*x - mz;
        g(3) = x*x/((a-d)*(a-d)) + y*y/((b-d)*(b-d)) + z*z/((c-d)*(c-d)) - 1.0;
        if (fingertip.model.stiffnessType == StiffnessType::Hooke)
            g(4) = -fb.dot(n) - E[0]*d;
        else if (fingertip.model.stiffnessType == StiffnessType::Quadratic)
            g(4) = -fb.dot(n) - E[0]*d - E[1]*d*d;
        else
            g(4) = 0.0;
    };
    auto evalJ = [&]() {
        J(0,0) = 2*k/((a-d)*(a-d)); J(0,1) = fz;               J(0,2) = -fy;
        J(0,3) = 2*x/((a-d)*(a-d)); J(0,4) = 4*k*x/((a-d)*(a-d)*(a-d));

        J(1,0) = -fz;                J(1,1) = 2*k/((b-d)*(b-d)); J(1,2) = fx;
        J(1,3) = 2*y/((b-d)*(b-d)); J(1,4) = 4*k*y/((b-d)*(b-d)*(b-d));

        J(2,0) = fy;                 J(2,1) = -fx;               J(2,2) = 2*k/((c-d)*(c-d));
        J(2,3) = 2*z/((c-d)*(c-d)); J(2,4) = 4*k*z/((c-d)*(c-d)*(c-d));

        J(3,0) = 2*x/((a-d)*(a-d)); J(3,1) = 2*y/((b-d)*(b-d)); J(3,2) = 2*z/((c-d)*(c-d));
        J(3,3) = 0.0;
        J(3,4) = 2*x*x/((a-d)*(a-d)*(a-d)) + 2*y*y/((b-d)*(b-d)*(b-d)) + 2*z*z/((c-d)*(c-d)*(c-d));

        J(4,0) = -2*fx/(a*a); J(4,1) = -2*fy/(b*b); J(4,2) = -2*fz/(c*c); J(4,3) = 0.0;
        if (fingertip.model.stiffnessType == StiffnessType::Hooke)
            J(4,4) = -E[0];
        else if (fingertip.model.stiffnessType == StiffnessType::Quadratic)
            J(4,4) = -E[0] - 2*E[1]*d;
        else
            J(4,4) = 0.0;
    };

    evalG();
    double Xi = g.squaredNorm(), Xi_old = Xi;
    int count = 0;

    while (Xi >= stop_th && count <= count_max) {
        evalJ();
        h = -(J.transpose() * J + lambda * I).inverse() * J.transpose() * g;
        x += h(0); y += h(1); z += h(2); k += h(3); d += h(4);
        evalG();

        if (verbose) {
            const Eigen::IOFormat fmt(5, Eigen::DontAlignCols, "\t|", " ");
            std::cout << "SITS LM Step " << count << "  g=" << g.transpose().format(fmt)
                      << "  x=[" << x << "," << y << "," << z << "," << k << "," << d << "]\n";
        }

        Xi_old = Xi;
        Xi = g.squaredNorm();
        if ((Xi_old - Xi) > epsilon * h.transpose() * (lambda * h - J.transpose() * g))
            lambda /= 10.0;
        else
            lambda *= 10.0;
        ++count;
    }

    X.c = {x, y, z}; X.K = k; X.Dd = d;
    return count;
}

// ============================================================================
//  Gauss-Newton  (analytic surface)
// ============================================================================
int SoftIntrinsicTactileSensing::solveContactSensingProblemGN(
        ContactSensingProblemSolution X0, Eigen::Vector3d f_in, Eigen::Vector3d m_in,
        double forceThreshold, int count_max, double stop_th, double epsilon, bool verbose) {

    this->f = f_in; this->m = m_in;
    if (f_in.norm() < forceThreshold) return -1;

    const double a = fingertip.model.principalAxisCoeff[0];
    const double b = fingertip.model.principalAxisCoeff[1];
    const double c = fingertip.model.principalAxisCoeff[2];
    const std::vector<double>& E = fingertip.model.stiffnessCoefficients;

    Eigen::Vector3d fb, mb;
    transformToFingertipFrame(f_in, m_in, fingertip.orientation, fingertip.displacement, fb, mb);
    this->f = fb; this->m = mb;

    const double fx = fb(0), fy = fb(1), fz = fb(2);
    const double mx = mb(0), my = mb(1), mz = mb(2);

    this->X = X0;
    double x = X.c(0), y = X.c(1), z = X.c(2), k = X.K, d = X.Dd;

    Eigen::VectorXd g(5);
    Eigen::MatrixXd J(5, 5);
    Eigen::VectorXd h(5);

    auto evalG = [&]() {
        Eigen::Vector3d n = fingertip.model.getNormal(x, y, z, d).normalized();
        g(0) = 2*k*x/((a-d)*(a-d)) - fy*z + fz*y - mx;
        g(1) = 2*k*y/((b-d)*(b-d)) - fz*x + fx*z - my;
        g(2) = 2*k*z/((c-d)*(c-d)) - fx*y + fy*x - mz;
        g(3) = x*x/((a-d)*(a-d)) + y*y/((b-d)*(b-d)) + z*z/((c-d)*(c-d)) - 1.0;
        if (fingertip.model.stiffnessType == StiffnessType::Hooke)
            g(4) = -fb.dot(n) - E[0]*d;
        else if (fingertip.model.stiffnessType == StiffnessType::Quadratic)
            g(4) = -fb.dot(n) - E[0]*d - E[1]*d*d;
        else
            g(4) = 0.0;
    };
    auto evalJ = [&]() {
        J(0,0) = 2*k/((a-d)*(a-d)); J(0,1) = fz;               J(0,2) = -fy;
        J(0,3) = 2*x/((a-d)*(a-d)); J(0,4) = 4*k*x/((a-d)*(a-d)*(a-d));

        J(1,0) = -fz;                J(1,1) = 2*k/((b-d)*(b-d)); J(1,2) = fx;
        J(1,3) = 2*y/((b-d)*(b-d)); J(1,4) = 4*k*y/((b-d)*(b-d)*(b-d));

        J(2,0) = fy;                 J(2,1) = -fx;               J(2,2) = 2*k/((c-d)*(c-d));
        J(2,3) = 2*z/((c-d)*(c-d)); J(2,4) = 4*k*z/((c-d)*(c-d)*(c-d));

        J(3,0) = 2*x/((a-d)*(a-d)); J(3,1) = 2*y/((b-d)*(b-d)); J(3,2) = 2*z/((c-d)*(c-d));
        J(3,3) = 0.0;
        J(3,4) = 2*x*x/((a-d)*(a-d)*(a-d)) + 2*y*y/((b-d)*(b-d)*(b-d)) + 2*z*z/((c-d)*(c-d)*(c-d));

        J(4,0) = -2*fx/(a*a); J(4,1) = -2*fy/(b*b); J(4,2) = -2*fz/(c*c); J(4,3) = 0.0;
        if (fingertip.model.stiffnessType == StiffnessType::Hooke)
            J(4,4) = -E[0];
        else if (fingertip.model.stiffnessType == StiffnessType::Quadratic)
            J(4,4) = -E[0] - 2*E[1]*d;
        else
            J(4,4) = 0.0;
    };

    evalG();
    double Xi = g.squaredNorm();
    int count = 0;

    while (Xi >= stop_th && count <= count_max) {
        evalJ();
        h = -epsilon * J.transpose() * g;
        x += h(0); y += h(1); z += h(2); k += h(3); d += h(4);
        evalG();

        if (verbose) {
            const Eigen::IOFormat fmt(5, Eigen::DontAlignCols, "\t|", " ");
            std::cout << "SITS GN Step " << count << "  g=" << g.transpose().format(fmt) << "\n";
        }

        Xi = g.squaredNorm();
        ++count;
    }

    X.c = {x, y, z}; X.K = k; X.Dd = d;
    return count;
}

// ============================================================================
//  Closed-Form  (shrunk ellipsoid, from stiffness model)
// ============================================================================
int SoftIntrinsicTactileSensing::solveContactSensingProblemCF(
        Eigen::Vector3d f_in, Eigen::Vector3d m_in, double forceThreshold) {

    this->f = f_in; this->m = m_in;
    if (f_in.norm() < forceThreshold) return -1;

    const double a = fingertip.model.principalAxisCoeff[0];
    const double b = fingertip.model.principalAxisCoeff[1];
    const double c = fingertip.model.principalAxisCoeff[2];
    const std::vector<double>& E = fingertip.model.stiffnessCoefficients;

    Eigen::Vector3d p, t;
    transformToFingertipFrame(f_in, m_in, fingertip.orientation, fingertip.displacement, p, t);
    this->f = p; this->m = t;

    // ---- compute shrunk semi-axes from stiffness model --------------------
    Eigen::Vector3d psa;
    switch (fingertip.model.stiffnessType) {
        case StiffnessType::Hooke:
            psa = {a - std::abs(p(2))/E[0],
                   b - std::abs(p(2))/E[0],
                   c - std::abs(p(2))/E[0]};
            break;
        case StiffnessType::Quadratic:
        {
            const double Dd = (-E[0] + std::sqrt(E[0]*E[0] + 4*E[1]*std::abs(p(2)))) / (2*E[1]);
            psa = {a - Dd, b - Dd, c - Dd};
            break;
        }
        default:
            psa = {a, b, c};
            break;
    }

    // ---- Closed-Form on shrunk ellipsoid ----------------------------------
    double alpha = psa(0), beta = psa(1), gamma = psa(2), invR = 1.0;
    while (alpha < 1.0 || beta < 1.0 || gamma < 1.0) {
        invR *= 10.0;
        alpha = psa(0) * invR;
        beta  = psa(1) * invR;
        gamma = psa(2) * invR;
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
    const double pdott = p.dot(t);
    const double sigma = D*D * invAt.squaredNorm() - R*R * Ap.squaredNorm();

    const double K = (-(pdott / std::abs(pdott)) / (std::sqrt(2.0) * R * D))
                   * std::sqrt(sigma + std::sqrt(sigma*sigma + 4.0*D*D*R*R*pdott*pdott));
    const double detG = K * (K*K*D*D + Ap.squaredNorm());

    if (K == 0.0)
        return solveContactSensingProblemCF(f_in, m_in, forceThreshold);

    Eigen::Vector3d PoC = (1.0 / detG) *
                          (K*K*D*D * A.inverse() * A.inverse() * t
                           + K * AAp.cross(t)
                           + pdott * p);

    X.c  = PoC;
    X.K  = K;
    X.Dd = a - psa(0);   // deformation = original - shrunk semi-axis
    return 1;
}

// ============================================================================
//  Wrench Method  (to do)
// ============================================================================
int SoftIntrinsicTactileSensing::solveContactSensingProblemWM(
        Eigen::Vector3d, Eigen::Vector3d, double) {
    return -1;
}

// ============================================================================
//  Extended solution
// ============================================================================
ExtendedContactSensingProblemSolution SoftIntrinsicTactileSensing::getExtendedSolution() {
    ExtendedContactSensingProblemSolution csps;
    Eigen::Vector3d n = fingertip.model.getNormal(X.c(0), X.c(1), X.c(2), X.Dd).normalized();
    csps.PoC = X.c;
    csps.n   = n;
    csps.fn  = this->f.dot(n);
    csps.ft  = this->f - csps.fn * n;
    csps.t   = (X.K * fingertip.model.getNormal(X.c(0), X.c(1), X.c(2), X.Dd)).norm();
    csps.Dd  = X.Dd;
    return csps;
}

// ============================================================================
//  levmar callback  –  analytic soft surface
// ============================================================================
void SoftIntrinsicTactileSensing::soft_contact_sensing_problem(
        double* x, double* g, int /*m*/, int /*n*/, void* data) {
    double input[11];
    std::memcpy(input, data, 11 * sizeof(double));
    const double fx=input[0], fy=input[1], fz=input[2];
    const double mx=input[3], my=input[4], mz=input[5];
    const double a=input[6],  b=input[7],  c=input[8];
    const double e1=input[9], e2=input[10];

    // x = [cx, cy, cz, K, Dd]
    const double ad = a - x[4], bd = b - x[4], cd = c - x[4];
    const double nx = 2*x[0]/(ad*ad), ny = 2*x[1]/(bd*bd), nz = 2*x[2]/(cd*cd);
    const double mag = std::sqrt(nx*nx + ny*ny + nz*nz);

    g[0] = 2*x[3]*x[0]/(ad*ad) - fy*x[2] + fz*x[1] - mx;
    g[1] = 2*x[3]*x[1]/(bd*bd) - fz*x[0] + fx*x[2] - my;
    g[2] = 2*x[3]*x[2]/(cd*cd) - fx*x[1] + fy*x[0] - mz;
    g[3] = x[0]*x[0]/(ad*ad) + x[1]*x[1]/(bd*bd) + x[2]*x[2]/(cd*cd) - 1.0;
    g[4] = -(fx*nx + fy*ny + fz*nz) / mag - e1*x[4] - e2*x[4]*x[4];
}

void SoftIntrinsicTactileSensing::jacobian_soft_contact_sensing_problem(
        double* x, double* jac, int /*m*/, int /*n*/, void* data) {
    double input[11];
    std::memcpy(input, data, 11 * sizeof(double));
    const double fx=input[0], fy=input[1], fz=input[2];
    const double a=input[6],  b=input[7],  c=input[8];
    const double e1=input[9], e2=input[10];

    const double ad = a - x[4], bd = b - x[4], cd = c - x[4];

    // Row 0: dg0/d[x,y,z,K,Dd]
    jac[0]  = 2*x[3]/(ad*ad); jac[1]  = fz;              jac[2]  = -fy;
    jac[3]  = 2*x[0]/(ad*ad); jac[4]  = 4*x[3]*x[0]/(ad*ad*ad);
    // Row 1
    jac[5]  = -fz;             jac[6]  = 2*x[3]/(bd*bd);  jac[7]  = fx;
    jac[8]  = 2*x[1]/(bd*bd); jac[9]  = 4*x[3]*x[1]/(bd*bd*bd);
    // Row 2
    jac[10] = fy;              jac[11] = -fx;              jac[12] = 2*x[3]/(cd*cd);
    jac[13] = 2*x[2]/(cd*cd); jac[14] = 4*x[3]*x[2]/(cd*cd*cd);
    // Row 3
    jac[15] = 2*x[0]/(ad*ad); jac[16] = 2*x[1]/(bd*bd);  jac[17] = 2*x[2]/(cd*cd);
    jac[18] = 0.0;
    jac[19] = 2*x[0]*x[0]/(ad*ad*ad) + 2*x[1]*x[1]/(bd*bd*bd) + 2*x[2]*x[2]/(cd*cd*cd);
    // Row 4: approximated (original code uses non-deformed normal for dg4/d[x,y,z])
    jac[20] = -2*fx/(a*a); jac[21] = -2*fy/(b*b); jac[22] = -2*fz/(c*c);
    jac[23] = 0.0;
    jac[24] = -e1 - 2*e2*x[4];
}

// ============================================================================
//  levmar callback  –  mesh/convex-hull soft path (numerical Jacobian)
//  Data layout: [fx,fy,fz, mx,my,mz, e1, e2, surf_ptr(2 doubles)]
// ============================================================================
void SoftIntrinsicTactileSensing::soft_contact_sensing_problem_mesh(
        double* x, double* g, int /*m*/, int /*n*/, void* data) {
    double input[12];
    std::memcpy(input, data, 12 * sizeof(double));
    const double fx=input[0], fy=input[1], fz=input[2];
    const double mx=input[3], my=input[4], mz=input[5];
    const double e1=input[6], e2=input[7];

    const Surface* surf = nullptr;
    std::memcpy(&surf, &input[8], sizeof(Surface*));

    // x = [cx, cy, cz, K, Dd]
    Eigen::Vector3d c{x[0], x[1], x[2]};
    const double K  = x[3];
    const double Dd = x[4];
    Eigen::Vector3d fb{fx, fy, fz};
    Eigen::Vector3d mb{mx, my, mz};

    Eigen::Vector3d n    = surf->getNormal(c.x(), c.y(), c.z(), Dd).normalized();
    Eigen::Vector3d Kn   = K * n;
    Eigen::Vector3d res  = Kn - (mb - c.cross(fb));
    g[0] = res(0); g[1] = res(1); g[2] = res(2);

    // surface constraint
    g[3] = surf->evaluate(c, Dd);

    // force-deformation
    g[4] = -fb.dot(n) - e1*Dd - e2*Dd*Dd;
}

// ============================================================================
//  Optimised LM via levmar
// ============================================================================
int SoftIntrinsicTactileSensing::solveContactSensingProblemOptim(
        ContactSensingProblemSolution X0,
        Eigen::Vector3d f_in, Eigen::Vector3d m_in,
        its::OptimLM p) {

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
        const double a  = fingertip.model.principalAxisCoeff[0];
        const double b  = fingertip.model.principalAxisCoeff[1];
        const double c  = fingertip.model.principalAxisCoeff[2];
        const double e1 = fingertip.model.stiffnessCoefficients[0];
        const double e2 = fingertip.model.stiffnessCoefficients[1];

        double x[5] = {X0.c(0), X0.c(1), X0.c(2), X0.K, X0.Dd};
        double data[11] = {fb(0), fb(1), fb(2), mb(0), mb(1), mb(2), a, b, c, e1, e2};

        step = dlevmar_der(soft_contact_sensing_problem,
                           jacobian_soft_contact_sensing_problem,
                           x, nullptr, p.m, p.n, p.itmax,
                           p.opt, p.info, nullptr, nullptr, data);

        X.c = {x[0], x[1], x[2]}; X.K = x[3]; X.Dd = x[4];
    } else {
        // ---- mesh path: numerical Jacobian --------------------------------
        const double e1 = fingertip.model.stiffnessCoefficients[0];
        const double e2 = fingertip.model.stiffnessCoefficients[1];

        Eigen::Vector3d c0 = X0.c;
        if (c0.isZero(1e-9))
            c0 = fingertip.model.mesh.triangles[0].centroid();
        c0 = fingertip.model.projectOnSurface(c0);

        double x[5] = {c0(0), c0(1), c0(2), X0.K, X0.Dd};

        const Surface* surf_ptr = &fingertip.model;
        double data[12] = {fb(0), fb(1), fb(2), mb(0), mb(1), mb(2), e1, e2, 0.0, 0.0, 0.0, 0.0};
        std::memcpy(&data[8], &surf_ptr, sizeof(Surface*));

        step = dlevmar_dif(soft_contact_sensing_problem_mesh,
                           x, nullptr, p.m, p.n, p.itmax,
                           p.opt, p.info, nullptr, nullptr, data);

        X.c = {x[0], x[1], x[2]}; X.K = x[3]; X.Dd = x[4];
    }

    if (p.verbose) {
        std::cout << "[SITS] levmar stop reason : " << p.info[6]   << "\n"
                  << "[SITS] iterations         : " << p.info[5]   << "\n"
                  << "[SITS] ||e||_2 initial    : " << p.info[0]   << "\n"
                  << "[SITS] ||e||_2 final      : " << p.info[1]   << "\n"
                  << "[SITS] ||J^T e||_inf      : " << p.info[2]   << "\n"
                  << "[SITS] ||Δx||_2           : " << p.info[3]   << "\n"
                  << "[SITS] lambda final       : " << p.info[4]   << "\n";
    }
    return step;
}

// ============================================================================
//  setLMParameters  (5-dimensional problem)
// ============================================================================
void SoftIntrinsicTactileSensing::setLMParameters(double forceThreshold, int count_max,
                                                    double stop_th, double /*epsilon*/,
                                                    bool verbose) {
    params.force_threshold = forceThreshold;
    params.itmax           = count_max;
    params.m               = 5;
    params.n               = 5;
    params.verbose         = verbose;

    params.opt[0] = 1e2;
    params.opt[1] = 1e-15;
    params.opt[2] = 1e-15;
    params.opt[3] = stop_th;
}
