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
#include <cmath>
#include <limits>
#include <new>

namespace v4s12 {

namespace {

// Fixed part of a geometry record: hash (32), AABB min/max (6 doubles),
// vertex count, index count, quantization bits, payload size (4 x uint32).
constexpr uint64_t kGeometryRecordBytes = 32 + 6 * sizeof(double) + 4 * sizeof(uint32_t);

// Parses a complete .v4s buffer. Every size and offset is checked against
// the buffer before it is used; on any inconsistency it returns false.
bool parse_v4s(const char* data, uint64_t file_size, Header& hdr,
               std::vector<SceneNode>& nodes, std::vector<GeometryData>& geometries) {
    if (data == nullptr || file_size < sizeof(Header)) return false;

    // 1. Header
    std::memcpy(&hdr, data, sizeof(Header));
    static const char kMagic[4] = {'V', '4', 'S', '2'};
    if (std::memcmp(hdr.magic, kMagic, sizeof(kMagic)) != 0) return false;
    if (hdr.version != Header().version) return false;

    // 2. Scene nodes
    if (hdr.offset_hashtable > file_size) return false;
    const uint64_t nodes_bytes = static_cast<uint64_t>(hdr.num_nodes) * sizeof(SceneNode);
    if (nodes_bytes > file_size - hdr.offset_hashtable) return false;
    nodes.resize(hdr.num_nodes);
    if (nodes_bytes > 0) {
        std::memcpy(nodes.data(), data + hdr.offset_hashtable, static_cast<size_t>(nodes_bytes));
    }
    for (const SceneNode& n : nodes) {
        if (n.geometry_id >= hdr.num_geometries) return false;
    }

    // 3. Geometry records
    if (hdr.offset_geometries > file_size) return false;
    uint64_t pos = hdr.offset_geometries;
    if (hdr.num_geometries > (file_size - pos) / kGeometryRecordBytes) return false;
    geometries.resize(hdr.num_geometries);

    for (GeometryData& geom : geometries) {
        if (file_size - pos < kGeometryRecordBytes) return false;
        const char* p = data + pos;
        uint32_t payload_size = 0;
        std::memcpy(geom.hash.data(), p, 32);                                p += 32;
        std::memcpy(geom.local_aabb_min, p, 3 * sizeof(double));            p += 3 * sizeof(double);
        std::memcpy(geom.local_aabb_max, p, 3 * sizeof(double));            p += 3 * sizeof(double);
        std::memcpy(&geom.original_vertex_count, p, sizeof(uint32_t));      p += sizeof(uint32_t);
        std::memcpy(&geom.original_index_count, p, sizeof(uint32_t));       p += sizeof(uint32_t);
        std::memcpy(&geom.quantization_bits, p, sizeof(uint32_t));          p += sizeof(uint32_t);
        std::memcpy(&payload_size, p, sizeof(uint32_t));
        pos += kGeometryRecordBytes;

        if (geom.quantization_bits < 1 || geom.quantization_bits > 16) return false;
        if (geom.original_index_count % 3 != 0) return false;
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(geom.local_aabb_min[axis]) || !std::isfinite(geom.local_aabb_max[axis]) ||
                geom.local_aabb_min[axis] > geom.local_aabb_max[axis]) return false;
        }
        if (payload_size > file_size - pos) return false;
        const char* payload = data + pos;
        pos += payload_size;

        // The ZSTD frame must declare exactly the size implied by the counts.
        const uint64_t coord_count = static_cast<uint64_t>(geom.original_vertex_count) * 3;
        const uint64_t position_bytes = coord_count * sizeof(uint16_t);
        const uint64_t indices_bytes = static_cast<uint64_t>(geom.original_index_count) * sizeof(uint32_t);
        const uint64_t expected = position_bytes + indices_bytes;
        const unsigned long long frame_size = ZSTD_getFrameContentSize(payload, payload_size);
        if (frame_size == ZSTD_CONTENTSIZE_ERROR || frame_size == ZSTD_CONTENTSIZE_UNKNOWN ||
            frame_size != expected) return false;
        if (expected > std::numeric_limits<size_t>::max()) return false;

        std::vector<uint8_t> raw(static_cast<size_t>(expected));
        const size_t d_size = ZSTD_decompress(raw.data(), raw.size(), payload, payload_size);
        if (ZSTD_isError(d_size) || d_size != raw.size()) return false;

        // Indices must reference existing vertices.
        geom.indices.resize(geom.original_index_count);
        std::memcpy(geom.indices.data(), raw.data() + position_bytes, static_cast<size_t>(indices_bytes));
        for (uint32_t index : geom.indices) {
            if (index >= geom.original_vertex_count) return false;
        }

        // Dequantization: x = min + q / (2^bits - 1) * extent
        std::vector<uint16_t> quantized(static_cast<size_t>(coord_count));
        std::memcpy(quantized.data(), raw.data(), static_cast<size_t>(position_bytes));
        const double levels = static_cast<double>((1u << geom.quantization_bits) - 1u);
        geom.vertices.resize(static_cast<size_t>(coord_count));
        for (size_t k = 0; k < quantized.size(); ++k) {
            const size_t axis = k % 3;
            const double extent = geom.local_aabb_max[axis] - geom.local_aabb_min[axis];
            geom.vertices[k] = static_cast<float>(geom.local_aabb_min[axis] + (quantized[k] / levels) * extent);
        }
    }
    return true;
}

} // namespace

bool Decoder::read_memory(const char* data, size_t file_size) {
    header_ = Header();
    nodes_.clear();
    geometries_.clear();

    Header hdr;
    std::vector<SceneNode> nodes;
    std::vector<GeometryData> geometries;
    try {
        if (!parse_v4s(data, file_size, hdr, nodes, geometries)) return false;
    } catch (const std::bad_alloc&) {
        return false;  // declared sizes too large for available memory
    }

    // Only a fully valid file replaces the decoder state.
    header_ = hdr;
    nodes_ = std::move(nodes);
    geometries_ = std::move(geometries);
    return true;
}

bool Decoder::read_file(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return false;

    const std::streamsize size = file.tellg();
    if (size <= 0) return false;
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(static_cast<size_t>(size));
    if (!file.read(buffer.data(), size)) return false;

    return read_memory(buffer.data(), buffer.size());
}

} // namespace v4s12
