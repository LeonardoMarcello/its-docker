#ifndef FINGERTIP_SURFACE_H
#define FINGERTIP_SURFACE_H

// Included at the bottom of Fingertip.hpp.
// All MeshData and Surface method bodies live here so users need only include
// "its/Fingertip.hpp" without compiling a separate translation unit.

#include <fstream>
#include <sstream>
#include <iostream>
#include <cassert>
#include <cstring>
#include <set>
#include <cctype>
#include <algorithm>

#include <coal/BVH/BVH_model.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
namespace fcl = coal;


namespace fingertip {

// ============================================================================
//  MeshData :: buildBVH
// ============================================================================
inline void MeshData::buildBVH() {
    if (triangles.empty()) return;

    bvh_model = std::make_shared<fcl::BVHModel<fcl::OBBRSS>>();
    bvh_model->beginModel();
    for (const auto& tri : triangles)
        bvh_model->addTriangle(tri.v0, tri.v1, tri.v2);
    bvh_model->endModel();
}

// ============================================================================
//  MeshData :: load  (dispatch by extension)
// ============================================================================
inline bool MeshData::load(const std::string& filepath, double scale) {
    const auto dot = filepath.rfind('.');
    if (dot == std::string::npos) {
        std::cerr << "[Fingertip] load: no file extension in '" << filepath << "'\n";
        return false;
    }
    std::string ext = filepath.substr(dot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    if (ext == "obj") return loadOBJ(filepath, scale);
    if (ext == "stl") return loadSTL(filepath, scale);
    if (ext == "dae") return loadDAE(filepath, scale);

    std::cerr << "[Fingertip] load: unsupported extension '." << ext << "'\n";
    return false;
}


inline bool MeshData::loadOBJ(const std::string& filepath, double scale) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        std::cerr << "[Fingertip] Cannot open OBJ: " << filepath << "\n";
        return false;
    }
    vertices.clear();
    triangles.clear();

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string token;
        ss >> token;
        if (token == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            vertices.push_back({x*scale, y*scale, z*scale});
        } else if (token == "f") {
            // faces: "f i j k" or "f i/t/n j/t/n k/t/n"
            std::vector<int> idx;
            std::string s;
            while (ss >> s)
                idx.push_back(std::stoi(s) - 1);   // OBJ is 1-indexed
            // fan-triangulate
            for (std::size_t i = 1; i + 1 < idx.size(); ++i) {
                Triangle tri;
                tri.v0 = vertices.at(static_cast<std::size_t>(idx[0]));
                tri.v1 = vertices.at(static_cast<std::size_t>(idx[i]));
                tri.v2 = vertices.at(static_cast<std::size_t>(idx[i + 1]));
                triangles.push_back(tri);
            }
        }
    }
    std::cout << "[Fingertip] OBJ loaded: " << vertices.size()
              << " vertices, " << triangles.size() << " triangles.\n";
    if (!triangles.empty()) buildBVH();
    return !triangles.empty();
}
inline bool MeshData::loadSTL(const std::string& filepath, double scale) {
    vertices.clear();
    triangles.clear();

    // ---- try binary first --------------------------------------------------
    // Binary STL: 80-byte header, uint32 triangle count,
    //             then per-triangle: 12-byte normal + 3×12-byte vertex + 2-byte attr
    {
        std::ifstream bin(filepath, std::ios::binary);
        if (!bin.is_open()) {
            std::cerr << "[Fingertip] Cannot open STL: " << filepath << "\n";
            return false;
        }
        char header[80];
        bin.read(header, 80);

        uint32_t num_tri = 0;
        bin.read(reinterpret_cast<char*>(&num_tri), sizeof(uint32_t));
        const std::streampos expected_size =
            80 + 4 + static_cast<std::streampos>(num_tri) * 50;
        bin.seekg(0, std::ios::end);
        const std::streampos file_size = bin.tellg();

        if (file_size == expected_size) {
            bin.seekg(84);
            for (uint32_t t = 0; t < num_tri; ++t) {
                float n[3], v[9];
                bin.read(reinterpret_cast<char*>(n), 12);   // normal (unused)
                bin.read(reinterpret_cast<char*>(v), 36);   // 3 vertices
                uint16_t attr; bin.read(reinterpret_cast<char*>(&attr), 2);

                Triangle tri;
                // FIX: apply scale in binary path (was missing before)
                tri.v0 = Eigen::Vector3d(v[0]*scale, v[1]*scale, v[2]*scale);
                tri.v1 = Eigen::Vector3d(v[3]*scale, v[4]*scale, v[5]*scale);
                tri.v2 = Eigen::Vector3d(v[6]*scale, v[7]*scale, v[8]*scale);
                triangles.push_back(tri);
                vertices.push_back(tri.v0);
                vertices.push_back(tri.v1);
                vertices.push_back(tri.v2);
            }
            std::cout << "[Fingertip] STL (binary) loaded: "
                      << triangles.size() << " triangles.\n";
            buildBVH();
            return true;
        }
    }

    // ---- ASCII STL ---------------------------------------------------------
    {
        std::ifstream ifs(filepath);
        if (!ifs.is_open()) return false;
        std::string line;
        Triangle cur;
        int vcount = 0;
        while (std::getline(ifs, line)) {
            std::istringstream ss(line);
            std::string kw;
            ss >> kw;
            if (kw == "vertex") {
                double x, y, z;
                ss >> x >> y >> z;
                if      (vcount == 0) cur.v0 = {x*scale, y*scale, z*scale};
                else if (vcount == 1) cur.v1 = {x*scale, y*scale, z*scale};
                else if (vcount == 2) cur.v2 = {x*scale, y*scale, z*scale};
                ++vcount;
            } else if (kw == "endfacet") {
                if (vcount == 3) {
                    triangles.push_back(cur);
                    vertices.push_back(cur.v0);
                    vertices.push_back(cur.v1);
                    vertices.push_back(cur.v2);
                }
                vcount = 0;
            }
        }
    }
    std::cout << "[Fingertip] STL (ASCII) loaded: "
              << triangles.size() << " triangles.\n";
    if (!triangles.empty()) buildBVH();
    return !triangles.empty();
}
inline bool MeshData::loadDAE(const std::string& filepath, double scale) {
    vertices.clear();
    triangles.clear();

    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        std::cerr << "[Fingertip] Cannot open DAE: " << filepath << "\n";
        return false;
    }
    const std::string content((std::istreambuf_iterator<char>(ifs)),
                               std::istreambuf_iterator<char>());

    auto findTag = [&](const std::string& tag, std::size_t from = 0) -> std::size_t {
        return content.find(tag, from);
    };
    auto innerText = [&](const std::string& openTag, std::size_t from) -> std::string {
        std::size_t s = content.find('>', from);
        if (s == std::string::npos) return {};
        ++s;
        const std::string closeTag = "</" + openTag.substr(1);
        std::size_t e = content.find(closeTag, s);
        if (e == std::string::npos) return {};
        return content.substr(s, e - s);
    };
    auto parseFloats = [](const std::string& text) -> std::vector<double> {
        std::istringstream ss(text);
        std::vector<double> out;
        double v;
        while (ss >> v) out.push_back(v);
        return out;
    };
    auto parseInts = [](const std::string& text) -> std::vector<int> {
        std::istringstream ss(text);
        std::vector<int> out;
        int v;
        while (ss >> v) out.push_back(v);
        return out;
    };

    std::vector<double> positions;
    {
        std::size_t pos = 0;
        while ((pos = findTag("<float_array", pos)) != std::string::npos) {
            std::size_t idstart = content.find("id=\"", pos);
            std::size_t gt      = content.find('>', pos);
            if (idstart == std::string::npos || gt == std::string::npos) { ++pos; continue; }
            idstart += 4;
            std::size_t idend = content.find('"', idstart);
            const std::string id = content.substr(idstart, idend - idstart);

            const std::string idl = [&](){
                std::string tmp = id;
                std::transform(tmp.begin(), tmp.end(), tmp.begin(),
                               [](unsigned char c){ return std::tolower(c); });
                return tmp;
            }();
            if (idl.find("position") != std::string::npos ||
                idl.find("vertex")   != std::string::npos ||
                idl.find("mesh")     != std::string::npos) {
                const std::string txt = innerText("<float_array", pos);
                const auto vals = parseFloats(txt);
                if (vals.size() > positions.size())
                    positions = vals;
            }
            pos = gt + 1;
        }
    }
    if (positions.empty() || positions.size() % 3 != 0) {
        std::cerr << "[Fingertip] DAE: no usable <float_array> positions found.\n";
        return false;
    }
    for (std::size_t i = 0; i + 2 < positions.size(); i += 3)
        vertices.push_back({positions[i]*scale, positions[i+1]*scale, positions[i+2]*scale});

    auto buildFromIndices = [&](const std::vector<int>& idx,
                                int stride, int vertex_offset,
                                int vcount_per_face) -> bool {
        const int n = static_cast<int>(idx.size());
        for (int base = 0; base + stride * vcount_per_face - 1 < n;
             base += stride * vcount_per_face) {
            std::vector<int> vi;
            for (int v = 0; v < vcount_per_face; ++v)
                vi.push_back(idx[base + v * stride + vertex_offset]);
            for (int v = 1; v + 1 < vcount_per_face; ++v) {
                const std::size_t i0 = static_cast<std::size_t>(vi[0]);
                const std::size_t i1 = static_cast<std::size_t>(vi[v]);
                const std::size_t i2 = static_cast<std::size_t>(vi[v + 1]);
                if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size())
                    continue;
                Triangle tri;
                tri.v0 = vertices[i0]; tri.v1 = vertices[i1]; tri.v2 = vertices[i2];
                triangles.push_back(tri);
            }
        }
        return !triangles.empty();
    };

    {
        std::size_t pos = 0;
        while ((pos = findTag("<triangles", pos)) != std::string::npos) {
            std::size_t block_end = content.find("</triangles>", pos);
            if (block_end == std::string::npos) { ++pos; continue; }
            const std::string block = content.substr(pos, block_end - pos);
            int stride = 0, vertex_offset = 0;
            {
                std::size_t ip = 0;
                while ((ip = block.find("<input", ip)) != std::string::npos) {
                    ++stride;
                    std::size_t se = block.find("semantic=\"", ip);
                    std::size_t oe = block.find("offset=\"",   ip);
                    std::size_t np = block.find("<input", ip + 1);
                    if (se < np && se != std::string::npos) {
                        se += 10;
                        if (block.substr(se, 6) == "VERTEX") {
                            if (oe != std::string::npos && oe < np) {
                                oe += 8;
                                vertex_offset = std::stoi(block.substr(oe));
                            }
                        }
                    }
                    ip = block.find("<input", ip + 1);
                    if (ip == std::string::npos) break;
                }
            }
            if (stride == 0) stride = 1;
            std::size_t ps = block.find("<p>");
            if (ps == std::string::npos) { pos = block_end + 1; continue; }
            const std::string ptxt = block.substr(ps + 3,
                                       block.find("</p>", ps) - (ps + 3));
            buildFromIndices(parseInts(ptxt), stride, vertex_offset, 3);
            pos = block_end + 1;
        }
    }

    if (triangles.empty()) {
        std::size_t pos = 0;
        while ((pos = findTag("<polylist", pos)) != std::string::npos) {
            std::size_t block_end = content.find("</polylist>", pos);
            if (block_end == std::string::npos) { ++pos; continue; }
            const std::string block = content.substr(pos, block_end - pos);
            int stride = 0, vertex_offset = 0;
            {
                std::size_t ip = 0;
                while ((ip = block.find("<input", ip)) != std::string::npos) {
                    ++stride;
                    std::size_t se = block.find("semantic=\"", ip);
                    std::size_t oe = block.find("offset=\"",   ip);
                    std::size_t np = block.find("<input", ip + 1);
                    if (se < np && se != std::string::npos) {
                        se += 10;
                        if (block.substr(se, 6) == "VERTEX") {
                            if (oe != std::string::npos && oe < np) {
                                oe += 8;
                                vertex_offset = std::stoi(block.substr(oe));
                            }
                        }
                    }
                    ip = block.find("<input", ip + 1);
                    if (ip == std::string::npos) break;
                }
            }
            if (stride == 0) stride = 1;
            std::size_t vcs = block.find("<vcount>");
            std::size_t ps  = block.find("<p>");
            if (vcs == std::string::npos || ps == std::string::npos) {
                pos = block_end + 1; continue;
            }
            const std::string vctxt = block.substr(vcs + 8,
                                        block.find("</vcount>", vcs) - (vcs + 8));
            const auto vcounts = parseInts(vctxt);
            const std::string ptxt = block.substr(ps + 3,
                                       block.find("</p>", ps) - (ps + 3));
            const auto idx = parseInts(ptxt);
            int base = 0;
            for (int vc : vcounts) {
                std::vector<int> vi;
                for (int v = 0; v < vc; ++v)
                    vi.push_back(idx[base + v * stride + vertex_offset]);
                for (int v = 1; v + 1 < vc; ++v) {
                    const std::size_t i0 = static_cast<std::size_t>(vi[0]);
                    const std::size_t i1 = static_cast<std::size_t>(vi[v]);
                    const std::size_t i2 = static_cast<std::size_t>(vi[v + 1]);
                    if (i0 >= vertices.size() || i1 >= vertices.size() || i2 >= vertices.size())
                        continue;
                    Triangle tri;
                    tri.v0 = vertices[i0]; tri.v1 = vertices[i1]; tri.v2 = vertices[i2];
                    triangles.push_back(tri);
                }
                base += vc * stride;
            }
            pos = block_end + 1;
        }
    }

    if (triangles.empty()) {
        std::cerr << "[Fingertip] DAE: no triangles extracted.\n";
        return false;
    }
    std::cout << "[Fingertip] DAE loaded: " << vertices.size()
              << " vertices, " << triangles.size() << " triangles.\n";
    buildBVH();
    return true;
}
inline bool MeshData::loadXYZ(const std::string& filepath, double scale) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        std::cerr << "[Fingertip] Cannot open XYZ: " << filepath << "\n";
        return false;
    }
    vertices.clear();
    triangles.clear();

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        double x, y, z;
        if (ss >> x >> y >> z)
            vertices.push_back({x*scale, y*scale, z*scale});
    }
    std::cout << "[Fingertip] XYZ loaded: " << vertices.size() << " points.\n";
    return !vertices.empty();
}

// ============================================================================
//  MeshData :: buildConvexHull
//  Gift-wrapping / Jarvis march in 3-D.  O(n·h).
//  Calls buildBVH() on completion.
//
//  FIXES vs original:
//    A) Seed "sec" vertex: compare vertex X coordinate, not normalized dir X.
//    B) Seed third vertex: use signed volume (triple product) to stay in 3-D.
//    C) Seed edges: insert all six directed seed edges into `closed` upfront so
//       the main loop cannot re-grow the seed face from the back side.
//    D) Edge pushes after a winding-swapped triangle now match the actual winding.
// ============================================================================
inline void MeshData::buildConvexHull() {
    const std::size_t N = vertices.size();
    if (N < 4) {
        std::cerr << "[Fingertip] buildConvexHull: need ≥ 4 points.\n";
        return;
    }

    std::cout << "[Fingertip] building ConvexHull, it may take a while ...\n";
    triangles.clear();

    // ---- seed vertex: lowest z -----------------------------------------------
    std::size_t bot = 0;
    for (std::size_t i = 1; i < N; ++i)
        if (vertices[i].z() < vertices[bot].z()) bot = i;

    // FIX A: choose "sec" by largest X coordinate (not direction component)
    std::size_t sec = (bot == 0) ? 1 : 0;
    for (std::size_t i = 0; i < N; ++i) {
        if (i == bot) continue;
        if (vertices[i].x() > vertices[sec].x()) sec = i;
    }

    using Edge = std::pair<std::size_t, std::size_t>;
    std::vector<Edge> open;
    std::set<Edge>    closed;

    auto addEdge = [&](std::size_t a, std::size_t b) {
        if (!closed.count({b, a}))
            open.push_back({a, b});
    };

    // ---- seed triangle -------------------------------------------------------
    {
        std::size_t best = N;
        double bestVol = 0.0;

        const Eigen::Vector3d& vbot = vertices[bot];
        const Eigen::Vector3d& vsec = vertices[sec];
        const Eigen::Vector3d  edge = vsec - vbot;

        for (std::size_t i = 0; i < N; ++i) {
            if (i == bot || i == sec) continue;
            // FIX B: use signed volume (triple product) to pick the third seed vertex
            // We want the vertex that gives the most "outward" triangle: one with all
            // other points on the negative side of the plane (bot, sec, i).
            Eigen::Vector3d n_cand = edge.cross(vertices[i] - vbot);
            if (n_cand.norm() < 1e-12) continue;

            // Check that all other points are on the non-positive side
            bool valid = true;
            for (std::size_t j = 0; j < N; ++j) {
                if (j == bot || j == sec || j == i) continue;
                if (n_cand.dot(vertices[j] - vbot) > 1e-9) { valid = false; break; }
            }
            if (!valid) continue;

            // Among valid candidates pick the one giving the largest triangle area
            double area = n_cand.norm();
            if (area > bestVol) { bestVol = area; best = i; }
        }

        if (best == N) {
            std::cerr << "[Fingertip] buildConvexHull: could not find seed triangle.\n";
            return;
        }

        Triangle t0;
        t0.v0 = vbot; t0.v1 = vsec; t0.v2 = vertices[best];
        // Ensure outward normal points away from centroid
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& v : vertices) centroid += v;
        centroid /= static_cast<double>(N);
        if (t0.normal().dot(t0.centroid() - centroid) < 0)
            std::swap(t0.v1, t0.v2);
        triangles.push_back(t0);

        // FIX C: pre-seed all six directed edges into closed so the main loop
        //        cannot accidentally re-grow the seed triangle from the back.
        // Determine actual winding order indices after possible swap.
        std::size_t i0 = bot;
        std::size_t i1 = (t0.v1.isApprox(vsec)) ? sec  : best;
        std::size_t i2 = (t0.v1.isApprox(vsec)) ? best : sec;

        closed.insert({i0, i1}); closed.insert({i1, i2}); closed.insert({i2, i0});
        closed.insert({i1, i0}); closed.insert({i2, i1}); closed.insert({i0, i2});

        // Push the three outer edges (reversed winding = open side to grow from)
        addEdge(i1, i0);
        addEdge(i2, i1);
        addEdge(i0, i2);
    }

    // ---- expand --------------------------------------------------------------
    const int MAX_ITER = static_cast<int>(N) * 20;
    for (int iter = 0; !open.empty() && iter < MAX_ITER; ++iter) {
        auto [a, b] = open.back(); open.pop_back();
        if (closed.count({a, b})) continue;
        closed.insert({a, b});

        const Eigen::Vector3d& pa = vertices[a];
        const Eigen::Vector3d& pb = vertices[b];
        const Eigen::Vector3d  ab_vec = pb - pa;

        // Approximate outward direction: midpoint of edge away from global centroid
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& v : vertices) centroid += v;
        centroid /= static_cast<double>(N);
        Eigen::Vector3d outward = ((pa + pb) * 0.5 - centroid).normalized();

        std::size_t best      = N;
        double      bestScore = -1e9;

        for (std::size_t i = 0; i < N; ++i) {
            if (i == a || i == b) continue;
            Eigen::Vector3d n = ab_vec.cross(vertices[i] - pa);
            if (n.norm() < 1e-12) continue;
            n.normalize();

            // All other points must be on the non-positive side of plane (a,b,i)
            bool valid = true;
            for (std::size_t j = 0; j < N; ++j) {
                if (j == a || j == b || j == i) continue;
                if (n.dot(vertices[j] - pa) > 1e-9) { valid = false; break; }
            }
            if (!valid) continue;

            double score = n.dot(outward);
            if (score > bestScore) { bestScore = score; best = i; }
        }
        if (best == N) continue;

        Triangle tri;
        tri.v0 = pa; tri.v1 = pb; tri.v2 = vertices[best];

        // Ensure outward winding
        if (tri.normal().dot(tri.centroid() - centroid) < 0)
            std::swap(tri.v1, tri.v2);
        triangles.push_back(tri);

        // FIX D: push new open edges in the order matching the (possibly swapped) winding.
        // After the swap, tri.v1 and tri.v2 may have exchanged, so we derive i1,i2 from
        // the triangle's actual vertex values rather than from `best` directly.
        std::size_t i_mid  = tri.v1.isApprox(pb)         ? b    : best;
        std::size_t i_last = (i_mid == b)                 ? best : b;

        addEdge(i_mid,  a);       // edge from tri.v1 → tri.v0
        addEdge(i_last, i_mid);   // edge from tri.v2 → tri.v1
    }

    std::cout << "[Fingertip] ConvexHull: " << triangles.size() << " triangles.\n";
    buildBVH();
}

// ============================================================================
//  Surface :: pointTriangleDist
// ============================================================================
inline double Surface::pointTriangleDist(const Eigen::Vector3d& p,
                                         const Triangle& tri,
                                         Eigen::Vector3d& closest) {
    const Eigen::Vector3d& A = tri.v0, &B = tri.v1, &C = tri.v2;
    Eigen::Vector3d AB = B-A, AC = C-A, AP = p-A;
    double d1=AB.dot(AP), d2=AC.dot(AP);
    if (d1<=0&&d2<=0){closest=A;return (p-A).norm();}
    Eigen::Vector3d BP=p-B;
    double d3=AB.dot(BP),d4=AC.dot(BP);
    if(d3>=0&&d4<=d3){closest=B;return (p-B).norm();}
    Eigen::Vector3d CP=p-C;
    double d5=AB.dot(CP),d6=AC.dot(CP);
    if(d6>=0&&d5<=d6){closest=C;return (p-C).norm();}
    double vc=d1*d4-d3*d2;
    if(vc<=0&&d1>=0&&d3<=0){double v=d1/(d1-d3);closest=A+v*AB;return (p-closest).norm();}
    double vb=d5*d2-d1*d6;
    if(vb<=0&&d2>=0&&d6<=0){double w=d2/(d2-d6);closest=A+w*AC;return (p-closest).norm();}
    double va=d3*d6-d5*d4;
    if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0){
        double w=(d4-d3)/((d4-d3)+(d5-d6));
        closest=B+w*(C-B);return (p-closest).norm();
    }
    double den=1.0/(va+vb+vc), v=vb*den, w=vc*den;
    closest=A+v*AB+w*AC;
    return (p-closest).norm();
}

// ============================================================================
//  Surface :: closestTriangle
// ============================================================================
inline Surface::ClosestTriangleResult Surface::closestTriangle(const Eigen::Vector3d& p) const {
    // check if query has been cached
    if (p.isApprox(cache_query_point_, 1e-12)) {
        return cache_result_;
    }
    // Iterate through all triangles of the mesh to get closest one: O(N)
    double min_d = std::numeric_limits<double>::max();
    Eigen::Vector3d closest_p = p;
    size_t best_idx = 0;

    for (size_t i = 0; i < mesh.triangles.size(); ++i) {
        Eigen::Vector3d temp_p;
        double d = pointTriangleDist(p, mesh.triangles[i], temp_p);
        if (d < min_d) {
            min_d = d;
            closest_p = temp_p;
            best_idx = i;
        }
    }

    // Return result from closest triangle
    ClosestTriangleResult result;
    result.point = closest_p;
    result.normal = mesh.triangles[best_idx].normal();
    result.dist = min_d;
    result.index = best_idx;
    // cache query
    cache_query_point_ = p;
    cache_result_       = result;

    return result;
}

// ============================================================================
//  Compute Normal to the surface at a given point
//  Surface :: getNormal
//         * Dispatch solution based on surface type
// ============================================================================
inline Eigen::Vector3d Surface::getNormal(double x, double y, double z,
                                          double d) const {
    switch (surfaceType) {
        case SurfaceType::Sphere:
        case SurfaceType::Ellipsoid:  return normalEllipsoid(x, y, z, d);
        case SurfaceType::Mesh:
        case SurfaceType::ConvexHull: return normalMesh(x, y, z);
        default:                      return {0.0, 0.0, 1.0};
    }
}

// ============================================================================
//  Surface :: normalEllipsoid
// ============================================================================
inline Eigen::Vector3d Surface::normalEllipsoid(double x, double y, double z,
                                                 double d) const {
    const double a = principalAxisCoeff[0] - d;
    const double b = principalAxisCoeff[1] - d;
    const double c = principalAxisCoeff[2] - d;
    return {2.0*x/(a*a), 2.0*y/(b*b), 2.0*z/(c*c)};
}
// ============================================================================
//  Surface :: normalMesh
// ============================================================================
inline Eigen::Vector3d Surface::normalMesh(double x, double y, double z) const {
    return closestTriangle(Eigen::Vector3d{x, y, z}).normal;
}


// ============================================================================
//  Evaluate implicit function S(x,y,z).
//  Point on the surface has S=0, inside S<0, outside S>0
//
// Surface :: evaluate
// ============================================================================
inline double Surface::evaluate(const Eigen::Vector3d& p, double d) const {
    switch (surfaceType) {
        case SurfaceType::Sphere:
        case SurfaceType::Ellipsoid: {
            const double a = principalAxisCoeff[0]-d;
            const double b = principalAxisCoeff[1]-d;
            const double c = principalAxisCoeff[2]-d;
            return p.x()*p.x()/(a*a)+p.y()*p.y()/(b*b)+p.z()*p.z()/(c*c)-1.0;
        }
        case SurfaceType::Mesh:
        case SurfaceType::ConvexHull: return closestTriangle(p).dist;
        default:                      return p.z()-principalAxisCoeff[0];
    }
}

// ============================================================================
//  Surface :: projectOnSurface
// ============================================================================
inline Eigen::Vector3d Surface::projectOnSurface(const Eigen::Vector3d& p,
                                                  double d, int maxIter,
                                                  double tol) const {
    if (surfaceType == SurfaceType::Mesh ||
        surfaceType == SurfaceType::ConvexHull)
        return closestTriangle(p).point;

    Eigen::Vector3d q = p;
    for (int i = 0; i < maxIter; ++i) {
        double S = evaluate(q, d);
        if (std::abs(S) < tol) break;
        Eigen::Vector3d n = getNormal(q.x(), q.y(), q.z(), d);
        double nSq = n.squaredNorm();
        if (nSq < 1e-20) break;
        q -= (S / nSq) * n;
    }
    return q;
}

}  // namespace fingertip
#endif  // FINGERTIP_SURFACE_H
