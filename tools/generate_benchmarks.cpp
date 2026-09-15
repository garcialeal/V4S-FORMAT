/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * V4S-FORMAT / LIBV4S12 - Deterministic Vector Geometry Engine
 *
 * Copyright (C) 2026 Antonio García Leal <support@v4s12.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "v4s12/v4s12.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;

// 1. Base Cube Generator (For CAD block instancing)
void create_cube_mesh(std::vector<float>& vertices, 
                      std::vector<float>& normals, 
                      std::vector<uint32_t>& indices) {
    vertices = {
        -0.5f, -0.5f,  0.5f,   0.5f, -0.5f,  0.5f,   0.5f,  0.5f,  0.5f,  -0.5f,  0.5f,  0.5f,
        -0.5f, -0.5f, -0.5f,  -0.5f,  0.5f, -0.5f,   0.5f,  0.5f, -0.5f,   0.5f, -0.5f, -0.5f
    };

    normals = {
         0.0f,  0.0f,  1.0f,   0.0f,  0.0f,  1.0f,   0.0f,  0.0f,  1.0f,   0.0f,  0.0f,  1.0f,
         0.0f,  0.0f, -1.0f,   0.0f,  0.0f, -1.0f,   0.0f,  0.0f, -1.0f,   0.0f,  0.0f, -1.0f
    };

    indices = {
        0, 1, 2,  0, 2, 3,
        4, 5, 6,  4, 6, 7,
        4, 0, 3,  4, 3, 5,
        1, 7, 6,  1, 6, 2,
        3, 2, 6,  3, 6, 5,
        4, 7, 1,  4, 1, 0
    };
}

// 2. Dense Parametric Sphere Generator (SolidWorks / 3D Mesh)
void create_sphere_mesh(int rings, int sectors, 
                        std::vector<float>& vertices, 
                        std::vector<float>& normals, 
                        std::vector<uint32_t>& indices) {
    const float R = 1.0f / static_cast<float>(rings - 1);
    const float S = 1.0f / static_cast<float>(sectors - 1);
    const float PI = 3.14159265358979323846f;

    vertices.reserve(rings * sectors * 3);
    normals.reserve(rings * sectors * 3);

    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {
            float y = std::sin(-PI / 2.0f + PI * r * R);
            float x = std::cos(2.0f * PI * s * S) * std::sin(PI * r * R);
            float z = std::sin(2.0f * PI * s * S) * std::sin(PI * r * R);

            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);

            normals.push_back(x);
            normals.push_back(y);
            normals.push_back(z);
        }
    }

    indices.reserve(rings * sectors * 6);
    for (int r = 0; r < rings - 1; ++r) {
        for (int s = 0; s < sectors - 1; ++s) {
            indices.push_back(r * sectors + s);
            indices.push_back(r * sectors + (s + 1));
            indices.push_back((r + 1) * sectors + (s + 1));

            indices.push_back(r * sectors + s);
            indices.push_back((r + 1) * sectors + (s + 1));
            indices.push_back((r + 1) * sectors + s);
        }
    }
}

int main() {
    fs::create_directories("benchmarks");
    std::cout << "Generating benchmark datasets in ./benchmarks/...\n";

    // TEST 1: Massive AutoCAD Instancing (1 Geometry -> 10,000 Nodes)
    {
        v4s12::Encoder encoder;
        std::vector<float> verts, norms;
        std::vector<uint32_t> inds;
        create_cube_mesh(verts, norms, inds);

        uint32_t geom_id = encoder.add_geometry(verts, norms, inds);

        int grid_size = 100;
        for (int x = 0; x < grid_size; ++x) {
            for (int y = 0; y < grid_size; ++y) {
                double transform[16] = {
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    x * 2.0, y * 2.0, 0.0, 1.0
                };
                uint32_t color = (x * 255 / grid_size) << 24 | (y * 255 / grid_size) << 16 | 0x00FFFF;
                encoder.add_node(geom_id, transform, color, 0);
            }
        }
        encoder.write_file("benchmarks/cad_instanced_10k.v4s");
        std::cout << "[SUCCESS] Saved: benchmarks/cad_instanced_10k.v4s (10,000 scene nodes)\n";
    }

    // TEST 2: High-Density 3D Mesh (SolidWorks / Industrial CAD)
    {
        v4s12::Encoder encoder;
        std::vector<float> verts, norms;
        std::vector<uint32_t> inds;
        
        create_sphere_mesh(300, 300, verts, norms, inds);

        uint32_t geom_id = encoder.add_geometry(verts, norms, inds);
        
        double identity[16] = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        };
        encoder.add_node(geom_id, identity, 0x00FF00FF);
        encoder.write_file("benchmarks/dense_sphere_180k.v4s");
        std::cout << "[SUCCESS] Saved: benchmarks/dense_sphere_180k.v4s (180,000 triangles)\n";
    }

    return 0;
}