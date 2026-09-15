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

#ifndef V4S12_SPECTRAL_PIPELINE_H
#define V4S12_SPECTRAL_PIPELINE_H

#include "v4s12/types.h"

#ifdef __cplusplus
extern "C" {
#endif

void v4_transform_2d_4x4(v4s12_int_t block[16]);
void v4_zigzag_reorder(const v4s12_int_t in_block[16], v4s12_int_t out_vector[16]);
v4s12_int_t s12_quantize_spline(v4s12_int_t coord);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
namespace v4s12 {

// Inverse function prototypes (Decoder)
void v4_zigzag_reorder_inv(const v4s12_int_t in_vector[16], v4s12_int_t out_block[16]);
void v4_transform_2d_4x4_inv(v4s12_int_t block[16]);
v4s12_int_t s12_dequantize_spline(v4s12_int_t quantized_coord);

} // namespace v4s12
#endif

#endif // V4S12_SPECTRAL_PIPELINE_H