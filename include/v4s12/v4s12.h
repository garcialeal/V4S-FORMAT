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

#ifndef V4S12_H
#define V4S12_H

#include <stdint.h>
#include <stddef.h>

/* ========================================================================= */
/* .V4S CONTAINER AND SCENE LAYER (C++17)                                    */
/* ========================================================================= */

#ifdef __cplusplus

#include <vector>
#include <string>
#include <array>

#include "v4s12/types.h"
#include "g3_hash.h"

namespace v4s12 {

#pragma pack(push, 8)
struct Header {
    char magic[4] = {'V', '4', 'S', '2'};
    uint16_t version = 300;  /* format 3.0: uniform quantization */
    uint16_t flags = 0;
    
    double global_aabb_min[3] = {0.0, 0.0, 0.0};
    double global_aabb_max[3] = {0.0, 0.0, 0.0};

    uint32_t num_geometries = 0;
    uint32_t num_nodes = 0;
    
    uint64_t offset_hashtable = 0;
    uint64_t offset_geometries = 0;
};
#pragma pack(pop)

struct SceneNode {
    uint32_t geometry_id;
    std::array<double, 16> transform; 
    uint32_t layer_color_rgba;
    uint32_t node_flags;
};

class Encoder {
public:
    Encoder() = default;

    uint32_t add_geometry(const std::vector<float>& vertices,
                          const std::vector<float>& normals,
                          const std::vector<uint32_t>& indices);

    void add_node(uint32_t geometry_id, 
                  const double transform[16], 
                  uint32_t layer_color_rgba = 0xFFFFFFFF, 
                  uint32_t node_flags = 0);

    /* Bits per coordinate for position quantization (1..16, default 16).
     * Maximum error per axis = AABB extent / (2 * (2^bits - 1)).
     * Applies to geometries added after the call. */
    void set_quantization_bits(uint32_t bits);
    uint32_t quantization_bits() const { return quantization_bits_; }

    bool write_memory(std::vector<uint8_t>& out_buffer) const;
    bool write_file(const std::string& filepath);

private:
    std::vector<GeometryData> geometries_;
    std::vector<SceneNode> nodes_;
    uint32_t quantization_bits_ = 16;

    void calculate_aabb(const std::vector<float>& vertices, double min[3], double max[3]);
};

class Decoder {
public:
    Decoder() = default;
    
    bool read_file(const std::string& filename);
    bool read_memory(const char* data, size_t size);

    const std::vector<GeometryData>& get_geometries() const { return geometries_; }
    const std::vector<SceneNode>& get_nodes() const { return nodes_; }
    const Header& get_header() const { return header_; }

private:
    Header header_;
    std::vector<GeometryData> geometries_;
    std::vector<SceneNode> nodes_;
};

} // namespace v4s12

#endif // __cplusplus

#endif // V4S12_H