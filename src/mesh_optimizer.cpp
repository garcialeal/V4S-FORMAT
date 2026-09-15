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
 
#include "v4s12/mesh_optimizer.h"
#include "g3_hash.h" // Inclusion C++ (fuera de extern "C")
#include <unordered_map>
#include <tuple>
#include <cmath>
#include <cstring>

namespace v4s12 {

// Wrapper to inject G3-RH512-256 as the vertex hashing engine
struct G3SpatialHash {
    size_t operator()(const std::tuple<float, float, float>& vertex) const {
        // Pack truncated coordinates into a contiguous array
        float data[3] = { std::get<0>(vertex), std::get<1>(vertex), std::get<2>(vertex) };
        
        // Compute the 256-bit signature with native G3 algorithm
        Hash256 full_hash = compute_g3_hash(data, sizeof(data));
        
        // Extract the first 64 bits (size_t) for fast RAM mapping
        size_t quick_hash;
        std::memcpy(&quick_hash, full_hash.data(), sizeof(size_t));
        return quick_hash;
    }
};

void optimize_geometry(
    const std::vector<float>& in_vertices,
    const std::vector<uint32_t>& in_indices,
    std::vector<float>& out_vertices,
    std::vector<uint32_t>& out_indices) 
{
    out_vertices.clear();
    out_indices.clear();
    out_indices.reserve(in_indices.size());

    // Use G3 hash instead of the standard C++ library hash
    std::unordered_map<std::tuple<float, float, float>, uint32_t, G3SpatialHash> vertex_map;
    uint32_t current_new_index = 0;

    for (size_t i = 0; i < in_indices.size(); ++i) {
        uint32_t original_idx = in_indices[i];
        size_t v_offset = original_idx * 3;

        // Millimetric truncation to absorb AutoCAD drift before hashing
        auto v_tuple = std::make_tuple(
            std::round(in_vertices[v_offset] * 1e5f) / 1e5f,
            std::round(in_vertices[v_offset + 1] * 1e5f) / 1e5f,
            std::round(in_vertices[v_offset + 2] * 1e5f) / 1e5f
        );

        auto it = vertex_map.find(v_tuple);
        if (it == vertex_map.end()) {
            vertex_map[v_tuple] = current_new_index;
            out_vertices.push_back(in_vertices[v_offset]);
            out_vertices.push_back(in_vertices[v_offset + 1]);
            out_vertices.push_back(in_vertices[v_offset + 2]);
            
            out_indices.push_back(current_new_index);
            current_new_index++;
        } else {
            out_indices.push_back(it->second);
        }
    }
}

} // namespace v4s12