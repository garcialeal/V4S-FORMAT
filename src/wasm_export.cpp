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

#include <emscripten/bind.h>
#include "v4s12/v4s12.h"

using namespace emscripten;
using namespace v4s12;

// Direct exposure of C++ memory to JavaScript TypedArrays
val get_geometry_vertices(const Decoder& decoder, uint32_t index) {
    const auto& geoms = decoder.get_geometries();
    if (index >= geoms.size()) return val::null();
    return val(typed_memory_view(geoms[index].vertices.size(), geoms[index].vertices.data()));
}

val get_geometry_indices(const Decoder& decoder, uint32_t index) {
    const auto& geoms = decoder.get_geometries();
    if (index >= geoms.size()) return val::null();
    return val(typed_memory_view(geoms[index].indices.size(), geoms[index].indices.data()));
}

EMSCRIPTEN_BINDINGS(v4s12_wasm) {
    class_<Decoder>("Decoder")
        .constructor<>()
        // JavaScript passes the file binary as a string/Uint8Array
        .function("read_memory", optional_override([](Decoder& self, const std::string& buffer) {
            return self.read_memory(buffer.data(), buffer.size());
        }))
        .function("get_geometry_vertices", &get_geometry_vertices)
        .function("get_geometry_indices", &get_geometry_indices);
}