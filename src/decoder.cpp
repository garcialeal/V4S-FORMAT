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
#include "v4s12/types.h"
#include <fstream>
#include <cstring>
#include <zstd.h>
#include <iostream>
#include <vector>

namespace v4s12 {

bool Decoder::read_memory(const char* data, size_t file_size) {
    if (file_size < sizeof(Header)) return false;

    const char* ptr = data;

    // 1. Zero-Copy Header Reading
    Header hdr;
    std::memcpy(&hdr, ptr, sizeof(Header));
    ptr = data + hdr.offset_hashtable; // Direct jump to nodes

    // 2. Zero-Copy Scene Graph Reading
    nodes_.resize(hdr.num_nodes);
    size_t nodes_size = hdr.num_nodes * sizeof(SceneNode);
    std::memcpy(nodes_.data(), ptr, nodes_size);
    ptr = data + hdr.offset_geometries; // Direct jump to geometries

    geometries_.resize(hdr.num_geometries);

    for (uint32_t i = 0; i < hdr.num_geometries; ++i) {
        GeometryData& geom = geometries_[i];

        // Geometry metadata extraction
        std::memcpy(geom.hash.data(), ptr, 32); ptr += 32;
        std::memcpy(geom.local_aabb_min, ptr, 3 * sizeof(double)); ptr += 3 * sizeof(double);
        std::memcpy(geom.local_aabb_max, ptr, 3 * sizeof(double)); ptr += 3 * sizeof(double);
        std::memcpy(&geom.original_vertex_count, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
        std::memcpy(&geom.original_index_count, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
        std::memcpy(&geom.quantization_bits, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
        if (geom.quantization_bits < 1 || geom.quantization_bits > 16) return false;

        uint32_t payload_size;
        std::memcpy(&payload_size, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);

        // 3. ZSTD decompression of the payload
        const size_t coord_count = static_cast<size_t>(geom.original_vertex_count) * 3;
        const size_t position_bytes = coord_count * sizeof(uint16_t);
        const size_t indices_bytes = static_cast<size_t>(geom.original_index_count) * sizeof(uint32_t);
        const size_t uncompressed_size = position_bytes + indices_bytes;

        std::vector<uint8_t> raw_payload(uncompressed_size);
        size_t d_size = ZSTD_decompress(raw_payload.data(), uncompressed_size, ptr, payload_size);
        if (ZSTD_isError(d_size) || d_size != uncompressed_size) return false;
        ptr += payload_size;

        // 4. Dequantization: x = min + q / (2^bits - 1) * extent
        std::vector<uint16_t> quantized(coord_count);
        std::memcpy(quantized.data(), raw_payload.data(), position_bytes);
        geom.indices.resize(geom.original_index_count);
        std::memcpy(geom.indices.data(), raw_payload.data() + position_bytes, indices_bytes);

        const double levels = static_cast<double>((1u << geom.quantization_bits) - 1u);
        geom.vertices.resize(coord_count);
        for (size_t k = 0; k < coord_count; ++k) {
            const size_t axis = k % 3;
            const double extent = geom.local_aabb_max[axis] - geom.local_aabb_min[axis];
            geom.vertices[k] = static_cast<float>(geom.local_aabb_min[axis] + (quantized[k] / levels) * extent);
        }
    }
    return true;
}

bool Decoder::read_file(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return false;

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size)) return false;

    return read_memory(buffer.data(), size);
}

} // namespace v4s12