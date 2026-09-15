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
#include "v4s12/spectral_pipeline.h"
#include "v4_transform.h" // Fuera de extern "C" porque arrastra tipos C++

extern "C" {
#include "s12_quant.h"
#include "v4_zigzag.h"
}

// Resto del código...

namespace v4s12 {

// 1. Inverse ZigZag Reordering
void v4_zigzag_reorder_inv(const v4s12_int_t in_vector[16], v4s12_int_t out_block[16]) {
    // Restores Klein orbits to their 4x4 spatial coordinates
    for (size_t i = 0; i < 16; ++i) {
        out_block[V4_ZIGZAG_MAP[i]] = in_vector[i];
    }
}

// 2. Inverse Butterfly Transform (2D 4x4)
void v4_transform_2d_4x4_inv(v4s12_int_t block[16]) {
    // Exploits the self-involutive property of the matrix: H_V4 * H_V4 = 4 * I_4
    v4_transform_2d_4x4(block);

    // Exact bitwise arithmetic scaling to return to the spatial domain
    for (size_t i = 0; i < 16; ++i) {
        block[i] >>= 4;
    }
}

// 3. S12 Dequantization
v4s12_int_t s12_dequantize_spline(v4s12_int_t quantized_coord) {
    // Upon exiting the inverse transform, the integer already sits exactly 
    // on a valid node of the S12 lattice (coprime residues 1, 5, 7, 11).
    // Its mathematical value is whole, acting as a direct bridge
    // toward the floating-point denormalization of the Bounding Box.
    return quantized_coord;
}

} // namespace v4s12