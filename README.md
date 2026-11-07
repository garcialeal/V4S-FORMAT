# v4s — compact container for deduplicated 3D triangle meshes

[![CI](https://github.com/garcialeal/V4S-FORMAT/actions/workflows/ci.yml/badge.svg)](https://github.com/garcialeal/V4S-FORMAT/actions/workflows/ci.yml)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)

`.v4s` is a binary container for scenes made of many instances of a smaller
set of triangle meshes, as found in CAD and BIM assemblies. Each unique mesh
is stored once; scene nodes reference it with a 4x4 transform and a color.

- **Deduplication:** identical meshes (same vertices, same triangles) are
  stored once. Identity is the [RH512](https://github.com/garcialeal/HASH-G3-RH512-256)
  hash (validated in SMHasher3), confirmed by exact byte comparison.
- **Bounded lossy compression:** positions are quantized uniformly over each
  mesh's bounding box (1 to 16 bits per coordinate, default 16) and the
  payload is compressed with ZSTD. Maximum error per axis:
  `extent / (2 * (2^bits - 1))`.
- **Deterministic output:** the same scene always produces the same bytes.
- **Validated decoding:** every size and offset is checked; corrupted or
  truncated files are rejected, never read out of bounds.

Current format version: **3.0** (header `version = 300`). Files written by
earlier versions (v1.0 release, header 200) cannot be decoded.

---

## Measured results

`tools/generate_benchmarks` (Linux x86_64, ZSTD level 3, 16 bits):

| Dataset | Content | Raw size | `.v4s` size |
| :--- | :--- | ---: | ---: |
| `dense_sphere_180k` | 1 mesh: 89,110 vertices, 178,802 triangles | 3,214,944 B (float32 + uint32) | 1,211,155 B (37.7 %) |
| `cad_instanced_10k` | 1 cube mesh, 10,000 nodes | — | 1,440,284 B |

Round trip on `dense_sphere_180k`: maximum error 1.53e-5 and RMS error
8.6e-6 for a sphere of radius 1 (the maximum equals the theoretical bound).

Notes:
- Most of the compressed size of a dense mesh is the index buffer
  (32-bit indices); positions are the smaller part.
- Scene nodes are stored uncompressed (144 bytes each), so a scene with many
  instances is dominated by the node table.

---

## Encoding pipeline

    float32 positions + uint32 triangle indices
        │
        ├─ 1. Validation       sizes multiple of 3, finite values, indices in range
        ├─ 2. Deduplication    RH512 hash -> hash table -> exact byte comparison
        ├─ 3. Vertex welding   vertices whose coordinates round to the same
        │                      multiple of 1e-5 are merged; unused vertices dropped
        ├─ 4. Quantization     q = round((x - min) / extent * (2^bits - 1))
        └─ 5. ZSTD             quantized positions (uint16) + indices (uint32)

Vertex welding uses an absolute tolerance of 1e-5 model units and renumbers
vertices in order of first use. The decoded mesh has the same triangles,
possibly with fewer and reordered vertices. Normals passed to the encoder
are currently ignored (not stored).

---

## File format 3.0

All values are little-endian. The format targets little-endian hosts
(x86, ARM, WebAssembly); big-endian hosts are not supported.

    +--------------------------------------------------+
    | Header (80 bytes)                                |
    +--------------------------------------------------+
    | Scene nodes (num_nodes x 144 bytes)              |
    +--------------------------------------------------+
    | Geometry records (num_geometries, variable size) |
    +--------------------------------------------------+

### Header (80 bytes)

| Offset | Field | Type | Description |
| ---: | :--- | :--- | :--- |
| 0 | `magic` | `char[4]` | `'V' '4' 'S' '2'` |
| 4 | `version` | `uint16` | `300` |
| 6 | `flags` | `uint16` | `0x0006` (ZSTD payload, per-node colors) |
| 8 | `global_aabb_min` | `double[3]` | Reserved, currently written as zero |
| 32 | `global_aabb_max` | `double[3]` | Reserved, currently written as zero |
| 56 | `num_geometries` | `uint32` | Number of geometry records |
| 60 | `num_nodes` | `uint32` | Number of scene nodes |
| 64 | `offset_hashtable` | `uint64` | Byte offset of the scene node table |
| 72 | `offset_geometries` | `uint64` | Byte offset of the first geometry record |

### Scene node (144 bytes)

| Offset | Field | Type |
| ---: | :--- | :--- |
| 0 | `geometry_id` | `uint32` (index of a geometry record) |
| 4 | padding | 4 zero bytes |
| 8 | `transform` | `double[16]` (the tools in this repository use column-major order, translation in elements 12–14) |
| 136 | `layer_color_rgba` | `uint32` |
| 140 | `node_flags` | `uint32` |

### Geometry record (96 bytes + payload)

| Offset | Field | Type | Description |
| ---: | :--- | :--- | :--- |
| 0 | `hash` | `uint8[32]` | RH512 of the encoder input: `[bits][float count][index count][vertex bytes][index bytes]` |
| 32 | `local_aabb_min` | `double[3]` | Bounding box of the welded vertices |
| 56 | `local_aabb_max` | `double[3]` | |
| 80 | `vertex_count` | `uint32` | Vertices after welding |
| 84 | `index_count` | `uint32` | Multiple of 3 |
| 88 | `quantization_bits` | `uint32` | 1 to 16 |
| 92 | `payload_size` | `uint32` | Bytes of the ZSTD frame that follows |
| 96 | payload | ZSTD frame | Decompresses to `vertex_count * 3` `uint16` (x, y, z interleaved) followed by `index_count` `uint32` |

Dequantization: `x = min + q / (2^bits - 1) * (max - min)`.

---

## API

C++ (`include/v4s12/v4s12.h`):

    v4s12::Encoder enc;
    enc.set_quantization_bits(16);                     // optional, 1..16
    uint32_t id = enc.add_geometry(vertices, {}, indices);
    if (id == v4s12::Encoder::INVALID_ID) { /* invalid input */ }
    enc.add_node(id, transform);                       // returns false on unknown id
    enc.write_file("scene.v4s");

    v4s12::Decoder dec;
    if (dec.read_file("scene.v4s")) {
        const auto& geometries = dec.get_geometries(); // vertices, indices
        const auto& nodes = dec.get_nodes();
    }

A C API for WebAssembly, C#, Python and other bindings is in
`include/v4s12/v4s12_c_api.h`.

---

## Building and testing

Requirements: CMake 3.15+, a C99 and C++17 compiler, libzstd and pkg-config.

    git clone --recursive https://github.com/garcialeal/V4S-FORMAT.git
    cd V4S-FORMAT
    cmake -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build
    ctest --test-dir build --output-on-failure

Tools:

    ./build/generate_benchmarks          # writes benchmarks/*.v4s
    ./build/v4s_info benchmarks/dense_sphere_180k.v4s

The test suite checks the error bound at 16, 12 and 8 bits, deduplication,
byte-identical output, input validation, and that every truncated file and
2,000 randomly corrupted files are handled without crashing. CI runs it on
Linux (GCC, Clang), macOS, and with AddressSanitizer and
UndefinedBehaviorSanitizer.

Not covered by CI: Windows (the build finds zstd through pkg-config) and the
WebAssembly targets (`emcmake cmake`), which build `v4s12_decoder.js` and
`v4s12_encoder.js`.

---

## Changes from v1.0

Format 3.0 replaces the S12 lattice quantization and the V4 butterfly
transform used in v1.0. Measurements showed that the V4 transform roughly
tripled the compressed size of the vertex data and that S12 behaved like a
10-bit quantizer that compressed worse than a uniform one. Uniform 16-bit
quantization gives smaller files and an error about 80 times lower. It also
fixes deduplication (v1.0 ignored the triangles), non-deterministic node
padding and the lack of validation in the decoder.

---

## License

GPL-3.0-or-later. See [LICENSE](LICENSE). For licensing under other terms,
contact the author.
