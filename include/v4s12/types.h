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

#ifndef V4S12_TYPES_H
#define V4S12_TYPES_H

#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>

#ifndef V4S12_INT_T_DEFINED
#define V4S12_INT_T_DEFINED
#ifdef V4S12_HIGH_PRECISION
    typedef int64_t v4s12_int_t;
#else
    typedef int32_t v4s12_int_t;
#endif
#endif

namespace v4s12 {

enum PrimitiveTopology : uint16_t {
    TOPOLOGY_TRIANGLES    = 0,
    TOPOLOGY_LINES        = 1,
    TOPOLOGY_POINTS       = 2,
    TOPOLOGY_TETRAHEDRONS = 3,
    TOPOLOGY_VOXELS       = 4
};

enum AttributeSemantic : uint16_t {
    ATTR_POSITION = 0,
    ATTR_NORMAL   = 1,
    ATTR_COLOR    = 2,
    ATTR_SENSOR   = 3,
    ATTR_TENSOR   = 4
};

enum FormatFlags : uint16_t {
    FLAG_NONE            = 0x0000,
    FLAG_HAS_NORMALS     = 0x0001,
    FLAG_LAYER_COLORS    = 0x0002,
    FLAG_ZSTD_COMPRESSED = 0x0004, 
    FLAG_INDEX_16BIT     = 0x0008, 
    FLAG_PLANAR_2D       = 0x0010,
    FLAG_N_DIMENSIONAL   = 0x0020
};

enum StatusCode : int32_t {
    STATUS_SUCCESS          = 0,
    ERR_INVALID_MAGIC       = -1,
    ERR_UNSUPPORTED_VERSION = -2,
    ERR_HASH_COLLISION      = -3,
    ERR_CORRUPT_PAYLOAD     = -4,
    ERR_ZSTD_DECODE_FAIL    = -5
};

struct SpectralBlock {
    v4s12_int_t data[16]; 
};

struct MeshCluster {
    uint32_t vertex_offset;
    uint32_t index_offset;
    uint32_t element_count;
    float bounding_center[3]; 
    float bounding_radius;
};

struct GeometryData {
    std::array<uint8_t, 32> hash; 
    double local_aabb_min[3];
    double local_aabb_max[3];
    uint32_t original_vertex_count;
    uint32_t original_index_count;
    std::vector<uint8_t> compressed_payload;
    
    std::vector<float> vertices;
    std::vector<uint32_t> indices;
};

} // namespace v4s12

#endif // V4S12_TYPES_H