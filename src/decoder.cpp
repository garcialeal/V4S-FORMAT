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
#include "v4s12/spectral_pipeline.h"
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

        uint32_t payload_size;
        std::memcpy(&payload_size, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);

        // 3. On-Demand ZSTD Decompression (Payload Only)
        size_t padded_vertex_count = geom.original_vertex_count * 3;
        while (padded_vertex_count % 16 != 0) padded_vertex_count++;
        
        size_t spectral_bytes = padded_vertex_count * sizeof(v4s12_int_t);
        size_t indices_bytes = geom.original_index_count * sizeof(uint32_t);
        size_t uncompressed_size = spectral_bytes + indices_bytes;

        std::vector<uint8_t> raw_payload(uncompressed_size);
        size_t d_size = ZSTD_decompress(raw_payload.data(), uncompressed_size, ptr, payload_size);
        if (ZSTD_isError(d_size)) return false;
        ptr += payload_size;

        // 4. Mathematical Decoding: Inverse V4 + S12
        const v4s12_int_t* spectral_payload = reinterpret_cast<const v4s12_int_t*>(raw_payload.data());
        const uint32_t* indices_payload = reinterpret_cast<const uint32_t*>(raw_payload.data() + spectral_bytes);

        geom.indices.assign(indices_payload, indices_payload + geom.original_index_count);
        geom.vertices.reserve(geom.original_vertex_count * 3);

        double range[3] = {
            geom.local_aabb_max[0] - geom.local_aabb_min[0],
            geom.local_aabb_max[1] - geom.local_aabb_min[1],
            geom.local_aabb_max[2] - geom.local_aabb_min[2]
        };

        for (size_t b = 0; b < padded_vertex_count; b += 16) {
            v4s12_int_t zigzag_block[16];
            v4s12_int_t block[16];
            std::memcpy(zigzag_block, &spectral_payload[b], 16 * sizeof(v4s12_int_t));

            // Inverse Topo-Entropic Reordering
            v4_zigzag_reorder_inv(zigzag_block, block);
            
            // Inverse Spectral Butterfly
            v4_transform_2d_4x4_inv(block);

            // S12 Dequantization and Reprojection to Float32
            for (int k = 0; k < 16; ++k) {
                size_t v_idx = b + k;
                if (v_idx < geom.original_vertex_count * 3) {
                    int axis = v_idx % 3;
                    double normalized = s12_dequantize_spline(block[k]) / 4095.0;
                    float real_val = static_cast<float>(geom.local_aabb_min[axis] + (normalized * range[axis]));
                    geom.vertices.push_back(real_val);
                }
            }
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