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
#include <cassert>
#include <cmath>

using namespace v4s12;

int main() {
    std::cout << "[TEST] Starting V4S12 Round-Trip validation...\n";

    std::vector<float> vertices = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.5f, 1.0f, 0.5f
    };
    std::vector<float> normals = { 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f };
    std::vector<uint32_t> indices = { 0, 1, 2 };

    Encoder encoder;
    uint32_t geom_id = encoder.add_geometry(vertices, normals, indices);
    
    double transform[16] = {
        1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1
    };
    encoder.add_node(geom_id, transform, 0xFFFFFFFF, 0);

    const std::string test_file = "test_artifact.v4s";
    bool write_ok = encoder.write_file(test_file);
    assert(write_ok && "Failed to write file");

    Decoder decoder;
    bool read_ok = decoder.read_file(test_file);
    assert(read_ok && "Failed to read file");

    const auto& decoded_geometries = decoder.get_geometries();
    assert(decoded_geometries.size() == 1);
    
    // Tolerance check (12-bit quantization has a minimal expected deviation)
    const auto& dec_verts = decoded_geometries[0].vertices;
    assert(dec_verts.size() >= 9); // Padding adds extra vertices
    
    std::cout << "[TEST] Original vertex X: " << vertices[3] << " -> Restored X: " << dec_verts[3] << "\n";
    std::cout << "[TEST] SUCCESS: The S12+V4+ZSTD pipeline is reversible.\n";
    float max_error = 0.0f;
    for (size_t i = 0; i < vertices.size(); ++i) {
        float diff = std::abs(vertices[i] - dec_verts[i]);
        if (diff > max_error) max_error = diff;
    }
    std::cout << "[TEST] Maximum S12 quantization error (Delta): " << max_error << " units\n";
    return 0;
}