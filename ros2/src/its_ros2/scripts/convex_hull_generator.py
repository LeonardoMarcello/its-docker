#!/usr/bin/env python3
"""
stl_convex_hull.py  –  Compute the convex hull of an STL mesh and write it
                        back as a binary STL file.

Usage:
    python3 stl_convex_hull.py input.stl [output.stl]

If output.stl is omitted, the result is written next to the input file
with the suffix '_convex_hull.stl'.

Dependencies:
    pip install numpy scipy          # core
    pip install numpy-stl            # optional, only used for ASCII STL fallback

Examples:
    python3 stl_convex_hull.py ahand_ft_tip.stl
    python3 stl_convex_hull.py model.stl model_hull.stl
"""

import sys
import os
import struct
import numpy as np
from scipy.spatial import ConvexHull


# ── STL I/O ────────────────────────────────────────────────────────────────

def _read_binary_stl(filepath: str) -> np.ndarray:
    """Return (N*3, 3) array of all vertex positions from a binary STL."""
    with open(filepath, "rb") as f:
        f.read(80)                                      # header
        n_tri = struct.unpack("<I", f.read(4))[0]
        verts = []
        for _ in range(n_tri):
            f.read(12)                                  # skip stored normal
            for _ in range(3):
                verts.append(struct.unpack("<fff", f.read(12)))
            f.read(2)                                   # attribute byte count
    return np.array(verts, dtype=np.float64)


def _read_ascii_stl(filepath: str) -> np.ndarray:
    """Return (N*3, 3) array from an ASCII STL."""
    verts = []
    with open(filepath, "r") as f:
        for line in f:
            tok = line.split()
            if tok and tok[0] == "vertex":
                verts.append([float(tok[1]), float(tok[2]), float(tok[3])])
    return np.array(verts, dtype=np.float64)


def read_stl(filepath: str) -> np.ndarray:
    """Auto-detect binary vs ASCII STL and return all vertex positions."""
    with open(filepath, "rb") as f:
        header = f.read(80)
        n_tri  = struct.unpack("<I", f.read(4))[0]
        f.seek(0, 2)
        file_size = f.tell()

    expected_binary = 80 + 4 + n_tri * 50
    if file_size == expected_binary:
        return _read_binary_stl(filepath)
    else:
        return _read_ascii_stl(filepath)


def write_binary_stl(filepath: str,
                     triangles: list,
                     normals:   list) -> None:
    """Write a list of triangles (each a 3×3 array) as a binary STL."""
    with open(filepath, "wb") as f:
        f.write(b'\x00' * 80)                          # header
        f.write(struct.pack("<I", len(triangles)))
        for tri, n in zip(triangles, normals):
            f.write(struct.pack("<fff", *n))
            for v in tri:
                f.write(struct.pack("<fff", *v))
            f.write(b'\x00\x00')                       # attribute byte count


# ── Convex hull ─────────────────────────────────────────────────────────────

def compute_convex_hull_stl(input_path: str, output_path: str) -> None:
    print(f"[stl_convex_hull] Reading:  {input_path}")
    verts = read_stl(input_path)
    print(f"[stl_convex_hull] Vertices: {len(verts)} "
          f"(from {len(verts)//3} triangles)")

    hull     = ConvexHull(verts)
    centroid = verts[hull.vertices].mean(axis=0)

    triangles, normals = [], []
    for simplex in hull.simplices:
        v0 = verts[simplex[0]]
        v1 = verts[simplex[1]]
        v2 = verts[simplex[2]]

        # Outward face normal
        n = np.cross(v1 - v0, v2 - v0)
        length = np.linalg.norm(n)
        if length < 1e-10:
            continue                                    # degenerate triangle
        n /= length

        # Ensure consistent outward winding
        if np.dot(n, v0 - centroid) < 0:
            n  = -n
            v1, v2 = v2, v1

        triangles.append([v0, v1, v2])
        normals.append(n)

    print(f"[stl_convex_hull] Hull:     {len(triangles)} triangles "
          f"({len(hull.vertices)} unique vertices)")

    write_binary_stl(output_path, triangles, normals)
    print(f"[stl_convex_hull] Written:  {output_path}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_path = sys.argv[1]
    if not os.path.isfile(input_path):
        print(f"[stl_convex_hull] Error: file not found: {input_path}")
        sys.exit(1)

    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        base, _ = os.path.splitext(input_path)
        output_path = base + "_convex_hull.stl"

    compute_convex_hull_stl(input_path, output_path)


if __name__ == "__main__":
    main()
