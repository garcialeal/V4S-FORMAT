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

#include "v4s12/v4s12_c_api.h"
#include "v4s12/v4s12.h"
#include <cstring>
#include <vector>

struct v4s12_decoder_t {
    v4s12::Decoder cpp_decoder;
};

struct v4s12_encoder_t {
    v4s12::Encoder cpp_encoder;
    std::vector<uint8_t> encoded_buffer;
};

extern "C" {

/* ========================================================================= */
/* DECODER IMPLEMENTATION                                                    */
/* ========================================================================= */

v4s12_decoder_t* v4s12_decoder_create(void) {
    return new v4s12_decoder_t();
}

void v4s12_decoder_destroy(v4s12_decoder_t* decoder) {
    delete decoder;
}

int v4s12_decoder_load_memory(v4s12_decoder_t* decoder, const char* buffer, size_t size) {
    if (!decoder || !buffer || size == 0) return 0;
    return decoder->cpp_decoder.read_memory(buffer, size) ? 1 : 0;
}

uint32_t v4s12_decoder_get_geometry_count(const v4s12_decoder_t* decoder) {
    if (!decoder) return 0;
    return static_cast<uint32_t>(decoder->cpp_decoder.get_geometries().size());
}

uint32_t v4s12_decoder_get_node_count(const v4s12_decoder_t* decoder) {
    if (!decoder) return 0;
    return static_cast<uint32_t>(decoder->cpp_decoder.get_nodes().size());
}

int v4s12_decoder_get_geometry_info(const v4s12_decoder_t* decoder, uint32_t index, 
                                   uint32_t* out_vertex_count, uint32_t* out_index_count) {
    if (!decoder) return 0;
    
    const auto& geoms = decoder->cpp_decoder.get_geometries();
    if (index >= geoms.size()) return 0;
    
    const auto& g = geoms[index];
    if (out_vertex_count) *out_vertex_count = static_cast<uint32_t>(g.vertices.size() / 3);
    if (out_index_count) *out_index_count = static_cast<uint32_t>(g.indices.size());
    
    return 1;
}

int v4s12_decoder_read_geometry(const v4s12_decoder_t* decoder, uint32_t index,
                                float* out_vertices, uint32_t* out_indices) {
    if (!decoder) return 0;
    
    const auto& geoms = decoder->cpp_decoder.get_geometries();
    if (index >= geoms.size()) return 0;
    
    const auto& g = geoms[index];
    
    if (out_vertices && !g.vertices.empty()) {
        std::memcpy(out_vertices, g.vertices.data(), g.vertices.size() * sizeof(float));
    }
    if (out_indices && !g.indices.empty()) {
        std::memcpy(out_indices, g.indices.data(), g.indices.size() * sizeof(uint32_t));
    }
    
    return 1;
}

int v4s12_decoder_read_node(const v4s12_decoder_t* decoder, uint32_t index,
                            uint32_t* out_geom_id, double out_transform[16], uint32_t* out_color) {
    if (!decoder) return 0;
    
    const auto& nodes = decoder->cpp_decoder.get_nodes();
    if (index >= nodes.size()) return 0;
    
    const auto& n = nodes[index];
    
    if (out_geom_id) *out_geom_id = n.geometry_id;
    if (out_color) *out_color = n.layer_color_rgba;
    if (out_transform) {
        std::memcpy(out_transform, n.transform.data(), 16 * sizeof(double));
    }
    
    return 1;
}

/* ========================================================================= */
/* ENCODER IMPLEMENTATION                                                    */
/* ========================================================================= */

v4s12_encoder_t* v4s12_encoder_create(void) {
    return new v4s12_encoder_t();
}

void v4s12_encoder_destroy(v4s12_encoder_t* encoder) {
    delete encoder;
}

uint32_t v4s12_encoder_add_geometry(v4s12_encoder_t* encoder,
                                    const float* vertices, size_t vertex_count,
                                    const float* normals, size_t normal_count,
                                    const uint32_t* indices, size_t index_count) {
    if (!encoder || !vertices || vertex_count == 0 || !indices || index_count == 0) return 0xFFFFFFFF;

    std::vector<float> vec_vertices(vertices, vertices + vertex_count);
    std::vector<float> vec_normals;
    if (normals && normal_count > 0) {
        vec_normals.assign(normals, normals + normal_count);
    }
    std::vector<uint32_t> vec_indices(indices, indices + index_count);

    return encoder->cpp_encoder.add_geometry(vec_vertices, vec_normals, vec_indices);
}

void v4s12_encoder_add_node(v4s12_encoder_t* encoder, 
                            uint32_t geometry_id, 
                            const double transform[16], 
                            uint32_t layer_color_rgba, 
                            uint32_t node_flags) {
    if (!encoder || !transform) return;
    encoder->cpp_encoder.add_node(geometry_id, transform, layer_color_rgba, node_flags);
}

size_t v4s12_encoder_encode(v4s12_encoder_t* encoder) {
    if (!encoder) return 0;
    bool ok = encoder->cpp_encoder.write_memory(encoder->encoded_buffer);
    return ok ? encoder->encoded_buffer.size() : 0;
}

int v4s12_encoder_copy_buffer(v4s12_encoder_t* encoder, uint8_t* out_buffer, size_t buffer_size) {
    if (!encoder || !out_buffer || buffer_size < encoder->encoded_buffer.size()) return 0;
    std::memcpy(out_buffer, encoder->encoded_buffer.data(), encoder->encoded_buffer.size());
    return 1;
}

int v4s12_encoder_write_file(v4s12_encoder_t* encoder, const char* filepath) {
    if (!encoder || !filepath) return 0;
    return encoder->cpp_encoder.write_file(std::string(filepath)) ? 1 : 0;
}

} // extern "C"