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

#ifndef V4S12_C_API_H
#define V4S12_C_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/* DECODER C-API                                                             */
/* ========================================================================= */

typedef struct v4s12_decoder_t v4s12_decoder_t;

v4s12_decoder_t* v4s12_decoder_create(void);
void v4s12_decoder_destroy(v4s12_decoder_t* decoder);

int v4s12_decoder_load_memory(v4s12_decoder_t* decoder, const char* buffer, size_t size);

uint32_t v4s12_decoder_get_geometry_count(const v4s12_decoder_t* decoder);
uint32_t v4s12_decoder_get_node_count(const v4s12_decoder_t* decoder);

int v4s12_decoder_get_geometry_info(const v4s12_decoder_t* decoder, uint32_t index, 
                                   uint32_t* out_vertex_count, uint32_t* out_index_count);

int v4s12_decoder_read_geometry(const v4s12_decoder_t* decoder, uint32_t index,
                                float* out_vertices, uint32_t* out_indices);

int v4s12_decoder_read_node(const v4s12_decoder_t* decoder, uint32_t index,
                            uint32_t* out_geom_id, double out_transform[16], uint32_t* out_color);

/* ========================================================================= */
/* ENCODER C-API                                                             */
/* ========================================================================= */

typedef struct v4s12_encoder_t v4s12_encoder_t;

v4s12_encoder_t* v4s12_encoder_create(void);
void v4s12_encoder_destroy(v4s12_encoder_t* encoder);

uint32_t v4s12_encoder_add_geometry(v4s12_encoder_t* encoder,
                                    const float* vertices, size_t vertex_count,
                                    const float* normals, size_t normal_count,
                                    const uint32_t* indices, size_t index_count);

void v4s12_encoder_add_node(v4s12_encoder_t* encoder, 
                            uint32_t geometry_id, 
                            const double transform[16], 
                            uint32_t layer_color_rgba, 
                            uint32_t node_flags);

size_t v4s12_encoder_encode(v4s12_encoder_t* encoder);

int v4s12_encoder_copy_buffer(v4s12_encoder_t* encoder, uint8_t* out_buffer, size_t buffer_size);

int v4s12_encoder_write_file(v4s12_encoder_t* encoder, const char* filepath);

#ifdef __cplusplus
}
#endif

#endif // V4S12_C_API_H