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
 
#ifndef V4S12_MESH_OPTIMIZER_H
#define V4S12_MESH_OPTIMIZER_H

#include <vector>
#include <cstdint>

namespace v4s12 {

// Takes raw data exported from AutoCAD and returns a clean, indexed mesh
void optimize_geometry(
    const std::vector<float>& in_vertices,
    const std::vector<uint32_t>& in_indices,
    std::vector<float>& out_vertices,
    std::vector<uint32_t>& out_indices
);

} // namespace v4s12

#endif // V4S12_MESH_OPTIMIZER_H