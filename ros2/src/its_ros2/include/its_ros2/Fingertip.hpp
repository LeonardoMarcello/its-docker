#ifndef FINGERTIP_H
#define FINGERTIP_H

#include <string>
#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/LU>

#include <coal/BVH/BVH_model.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
namespace fcl = coal;

namespace fingertip {

// ============================================================================
//  Surface type enumeration
// ============================================================================
enum class SurfaceType : unsigned short int {
    Plane     = 1,   // S(x,y,z): z = R
    Sphere    = 2,   // S(x,y,z): x² + y² + z² = R²
    Cylinder  = 3,   // S(x,y,z): x²/a² + y²/b² = 1
    Ellipsoid = 4,   // S(x,y,z): x²/a² + y²/b² + z²/c² = 1
    Mesh      = 5,   // Arbitrary triangle mesh  (exact surface, BVH-accelerated)
    ConvexHull= 6,   // Convex hull of a point cloud / mesh vertices
};

// ============================================================================
//  Stiffness type enumeration
// ============================================================================
enum class StiffnessType : unsigned short int {
    Rigid     = 1,   // Not deformable
    Hooke     = 2,   // F = K · Δx
    Quadratic = 3,   // F = p₁·Δx + p₂·Δx²
    Power     = 4,   // F = a · Δx^b
};

// ============================================================================
//  Triangle  (used by Mesh / ConvexHull surfaces)
// ============================================================================
struct Triangle {
    Eigen::Vector3d v0, v1, v2;

    Eigen::Vector3d normal() const {
        return (v1 - v0).cross(v2 - v0).normalized();
    }
    Eigen::Vector3d centroid() const {
        return (v0 + v1 + v2) / 3.0;
    }
};

// ============================================================================
//  MeshData  – shared by Mesh and ConvexHull surface types
// ============================================================================
struct MeshData {
    std::vector<Eigen::Vector3d> vertices;   // raw vertices [mm]
    std::vector<Triangle>        triangles;  // triangle soup

    // FCL BVH model – built once after the triangle list is finalised
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> bvh_model;

    // approximation of mesh with ellipsoidal
    std::vector<double>      ProxyPrincipalAxisCoeff    {1.0, 1.0, 1.0};    // Principal axes (a,b,c) [mm]
    std::vector<double>      ProxyDisplacement    {0.0, 0.0, 0.0};          // mesh origin to Ellipsoid origin diplacement (x,y,z) [mm]
    std::vector<double>      ProxyOrientation    {0.0, 0.0, 0.0};           // mesh origin to Ellipsoid origin orientation (roll, pitch, yaw) [rad]


    bool empty() const { return triangles.empty(); }

    // ------------------------------------------------------------------
    //  Build the FCL OBBRSSd BVH acceleration structure.
    //  Must be called (once) after triangles is populated.
    // ------------------------------------------------------------------
    void buildBVH();

    // ------------------------------------------------------------------
    //  Build convex hull from current vertex set (gift-wrapping).
    //  Calls buildBVH() internally when done.
    // ------------------------------------------------------------------
    void buildConvexHull();

    // ------------------------------------------------------------------
    //  Fit the mesh with ellipsoidal geometriy TO DO.
    // ------------------------------------------------------------------
    void fitEllipsoidal();

    // ------------------------------------------------------------------
    //  Mesh loaders – each calls buildBVH() on success.
    // ------------------------------------------------------------------

    /** Dispatch by file extension: .obj → loadOBJ, .stl → loadSTL,
     *  .dae → loadDAE.  Returns false for unknown extensions. */
    bool load(const std::string& filepath, double scale = 1.0);

    /** Wavefront OBJ  – 'v' and 'f' tokens, fan-triangulation for n-gons. */
    bool loadOBJ(const std::string& filepath, double scale = 1.0);

    /** Binary or ASCII STL. */
    bool loadSTL(const std::string& filepath, double scale = 1.0);

    /** COLLADA DAE  – parses <float_array> geometry (no external deps). */
    bool loadDAE(const std::string& filepath, double scale = 1.0);

    /** XYZ point cloud (one point per line, space-separated).
     *  Triangle list is left empty; call buildConvexHull() afterwards. */
    bool loadXYZ(const std::string& filepath, double scale = 1.0);
};

// ============================================================================
//  Surface
// ============================================================================
class Surface {
public:
    // ---- attributes -------------------------------------------------------
    SurfaceType              surfaceType           {SurfaceType::Ellipsoid};
    std::vector<double>      principalAxisCoeff    {1.0, 1.0, 1.0};
    StiffnessType            stiffnessType         {StiffnessType::Rigid};
    std::vector<double>      stiffnessCoefficients {0.0, 0.0};

    MeshData                 mesh;   // populated only for Mesh / ConvexHull

    // ---- surface implicit function ----------------------------------------
    //  Analytic surfaces: S(p) = 0 on surface, > 0 outside.
    //  Mesh / ConvexHull: signed distance (negative = inside).
    double evaluate(const Eigen::Vector3d& p, double d = 0.0) const;

    // ---- outward surface normal  ------------------------------------------
    //  Analytic surfaces: exact gradient ∇S.
    //  Mesh / ConvexHull: normal of BVH-nearest triangle.
    //  d = uniform surface deformation [mm] (analytic surfaces only).
    Eigen::Vector3d getNormal(double x, double y, double z, double d = 0.0) const;

    // ---- project point onto surface  (Newton / BVH snap)  ----------------
    //  Returns the nearest point on the zero-level-set to query point p.
    Eigen::Vector3d projectOnSurface(const Eigen::Vector3d& p, double d = 0.0,
                                     int maxIter = 20, double tol = 1e-6) const;

    // ---- closest-triangle query  (BVH-accelerated) -----------------------
    struct ClosestTriangleResult {
        Eigen::Vector3d point;   // closest point on mesh surface
        Eigen::Vector3d normal;  // outward normal of the triangle
        double          dist;    // signed distance (negative = inside)
        std::size_t     index;   // triangle index in MeshData::triangles
    };
    ClosestTriangleResult closestTriangle(const Eigen::Vector3d& p) const;

private:
    Eigen::Vector3d normalEllipsoid(double x, double y, double z, double d) const;
    Eigen::Vector3d normalMesh(double x, double y, double z) const;

    // Fallback brute-force point-to-triangle (used only when BVH is absent)
    static double pointTriangleDist(const Eigen::Vector3d& p, const Triangle& tri,
                                    Eigen::Vector3d& closest);


    // ---- cache data to reduce getNormal query ---------------------------------------------
    mutable Eigen::Vector3d       cache_query_point_
        {Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
    mutable ClosestTriangleResult cache_result_;

};

// ============================================================================
//  Fingertip
// ============================================================================
class Fingertip {
public:
    std::string      id;
    Eigen::Matrix3d  orientation  {Eigen::Matrix3d::Identity()};
    Eigen::Vector3d  displacement {Eigen::Vector3d::Zero()};
    Surface          model;
};

}  // namespace fingertip

// ============================================================================
//  Inline / implementation section
// ============================================================================
#include "Fingertip_surface.hpp"

#endif  // FINGERTIP_H
