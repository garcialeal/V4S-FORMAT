# SPDX-License-Identifier: GPL-3.0-or-later
#
# V4S-FORMAT / LIBV4S12 - Deterministic Vector Geometry Engine
#
# Copyright (C) 2026 Antonio García Leal (garcialeal)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.15)
project(V4S12 VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(FetchContent)

# 1. ZSTD Dependency (Dynamic on PC, Compiled from source for WASM)
if(NOT EMSCRIPTEN)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ZSTD REQUIRED libzstd)
else()
    FetchContent_Declare(
        zstd
        GIT_REPOSITORY https://github.com/facebook/zstd.git
        GIT_TAG v1.5.5
        SOURCE_SUBDIR build/cmake
    )
    
    # Optimizations for a lightweight WASM binary
    set(ZSTD_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_STATIC ON CACHE BOOL "" FORCE)
    set(ZSTD_MULTITHREAD_SUPPORT OFF CACHE BOOL "" FORCE)
    
    FetchContent_MakeAvailable(zstd)
    
    set(ZSTD_LIBRARIES libzstd_static)
    set(ZSTD_INCLUDE_DIRS ${zstd_SOURCE_DIR}/lib)
endif()

# Core library pointing strictly to external submodules for math & hash
add_library(v4s12_lib STATIC
    src/encoder.cpp
    src/decoder.cpp
    src/mesh_optimizer.cpp
    src/spectral_pipeline.cpp
    src/v4s12_c_api.cpp
    external/HASH-G3-RH512-256/src/g3_hash.c
    external/V4S12-MATHEMATICAL-CORE/src/s12_quant.c
    external/V4S12-MATHEMATICAL-CORE/src/v4_transform.c
    external/V4S12-MATHEMATICAL-CORE/src/v4_zigzag.c
)

target_include_directories(v4s12_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/HASH-G3-RH512-256/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/V4S12-MATHEMATICAL-CORE/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/V4S12-MATHEMATICAL-CORE/include/v4s12
)

if(ZSTD_INCLUDE_DIRS)
    target_include_directories(v4s12_lib PUBLIC ${ZSTD_INCLUDE_DIRS})
endif()
if(ZSTD_LIBRARIES)
    target_link_libraries(v4s12_lib PUBLIC ${ZSTD_LIBRARIES})
endif()

# 2. Build Native Tools (Only if NOT building with Emscripten)
if(NOT EMSCRIPTEN)
    add_executable(test_v2 test/test_v2.cpp)
    target_link_libraries(test_v2 PRIVATE v4s12_lib)

    add_executable(v4s_info cli/v4s_info.cpp)
    target_link_libraries(v4s_info PRIVATE v4s12_lib)

    add_executable(generate_benchmarks tools/generate_benchmarks.cpp)
    target_link_libraries(generate_benchmarks PRIVATE v4s12_lib)
endif()

# =========================================================================
# WebAssembly / Emscripten Build Targets
# =========================================================================
if(EMSCRIPTEN)
    # --- DECODER WASM ---
    add_executable(v4s12_decoder_wasm src/v4s12_c_api.cpp)
    target_link_libraries(v4s12_decoder_wasm PRIVATE v4s12_lib)
    
    set_target_properties(v4s12_decoder_wasm PROPERTIES OUTPUT_NAME "v4s12_decoder")
    set_target_properties(v4s12_decoder_wasm PROPERTIES SUFFIX ".js")
    
    target_link_options(v4s12_decoder_wasm PRIVATE
        "-sEXPORTED_FUNCTIONS=['_v4s12_decoder_create','_v4s12_decoder_destroy','_v4s12_decoder_load_memory','_v4s12_decoder_get_geometry_count','_v4s12_decoder_get_node_count','_v4s12_decoder_get_geometry_info','_v4s12_decoder_read_geometry','_v4s12_decoder_read_node','_malloc','_free']"
        "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sMODULARIZE=1"
        "-sEXPORT_NAME='V4S12DecoderModule'"
    )

    # --- ENCODER WASM ---
    add_executable(v4s12_encoder_wasm src/v4s12_c_api.cpp)
    target_link_libraries(v4s12_encoder_wasm PRIVATE v4s12_lib)
    
    set_target_properties(v4s12_encoder_wasm PROPERTIES OUTPUT_NAME "v4s12_encoder")
    set_target_properties(v4s12_encoder_wasm PROPERTIES SUFFIX ".js")
    
    target_link_options(v4s12_encoder_wasm PRIVATE
        "-sEXPORTED_FUNCTIONS=['_v4s12_encoder_create','_v4s12_encoder_destroy','_v4s12_encoder_add_geometry','_v4s12_encoder_add_node','_v4s12_encoder_encode','_v4s12_encoder_copy_buffer','_malloc','_free']"
        "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sMODULARIZE=1"
        "-sEXPORT_NAME='V4S12EncoderModule'"
    )
endif()