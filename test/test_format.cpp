// SPDX-License-Identifier: GPL-3.0-or-later
//
// V4S-FORMAT test suite: round trip, deduplication, determinism,
// input validation and decoder robustness against corrupted files.
//
// Copyright (C) 2026 Antonio García Leal (garcialeal)

#include "v4s12/v4s12.h"
#include "v4s12/v4s12_c_api.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

using namespace v4s12;

static int failures = 0;

static void check(bool ok, const char* what) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

static const double kIdentity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

// Sphere without duplicated vertices (no welding needed).
static void make_sphere(int rings, int sectors, std::vector<float>& v, std::vector<uint32_t>& idx) {
    const double pi = 3.14159265358979323846;
    for (int r = 1; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {
            const double th = pi * r / rings, ph = 2 * pi * s / sectors;
            v.push_back(static_cast<float>(std::sin(th) * std::cos(ph)));
            v.push_back(static_cast<float>(std::cos(th)));
            v.push_back(static_cast<float>(std::sin(th) * std::sin(ph)));
        }
    }
    for (int r = 0; r < rings - 2; ++r) {
        for (int s = 0; s < sectors; ++s) {
            const uint32_t a = r * sectors + s, b = r * sectors + (s + 1) % sectors;
            const uint32_t c = a + sectors, d = b + sectors;
            idx.insert(idx.end(), {a, b, d, a, d, c});
        }
    }
}

static std::vector<uint8_t> encode_scene(uint32_t bits) {
    std::vector<float> v, n;
    std::vector<uint32_t> idx;
    make_sphere(24, 32, v, idx);
    Encoder enc;
    enc.set_quantization_bits(bits);
    const uint32_t id = enc.add_geometry(v, n, idx);
    enc.add_node(id, kIdentity, 0x11223344u, 7);
    std::vector<uint8_t> out;
    enc.write_memory(out);
    return out;
}

static void test_round_trip(uint32_t bits) {
    std::vector<float> v, n;
    std::vector<uint32_t> idx;
    make_sphere(24, 32, v, idx);

    Encoder enc;
    enc.set_quantization_bits(bits);
    const uint32_t id = enc.add_geometry(v, n, idx);
    enc.add_node(id, kIdentity);
    std::vector<uint8_t> buf;
    enc.write_memory(buf);

    Decoder dec;
    const bool ok = dec.read_memory(reinterpret_cast<const char*>(buf.data()), buf.size());
    char msg[128];
    std::snprintf(msg, sizeof(msg), "round trip decodes (%u bits)", bits);
    check(ok && dec.get_geometries().size() == 1, msg);
    if (!ok) return;

    // The mesh optimizer may renumber vertices, so compare triangle corners:
    // original point v[idx[k]] against decoded point vertices[indices[k]].
    const GeometryData& g = dec.get_geometries()[0];
    std::snprintf(msg, sizeof(msg), "same number of triangle corners (%u bits)", bits);
    check(g.indices.size() == idx.size(), msg);
    if (g.indices.size() != idx.size()) return;

    // Theoretical bound: extent / (2 (2^bits - 1)) per axis, plus float rounding.
    double worst_ratio = 0.0;
    for (size_t k = 0; k < idx.size(); ++k) {
        for (int axis = 0; axis < 3; ++axis) {
            const double original = v[3 * idx[k] + axis];
            const double decoded = g.vertices[3 * g.indices[k] + axis];
            const double extent = g.local_aabb_max[axis] - g.local_aabb_min[axis];
            const double bound = extent / (2.0 * ((1u << bits) - 1u)) + 1e-6;
            worst_ratio = std::fmax(worst_ratio, std::fabs(decoded - original) / bound);
        }
    }
    std::snprintf(msg, sizeof(msg), "every triangle corner within theoretical error bound (%u bits, worst %.2f of bound)", bits, worst_ratio);
    check(worst_ratio <= 1.0, msg);
}

static void test_deduplication() {
    std::vector<float> quad = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0}, n;
    Encoder enc;
    const uint32_t a = enc.add_geometry(quad, n, {0, 1, 2, 1, 3, 2});
    const uint32_t b = enc.add_geometry(quad, n, {0, 1, 3});
    const uint32_t c = enc.add_geometry(quad, n, {0, 1, 2, 1, 3, 2});
    check(a != b, "same vertices with different triangles are different geometries");
    check(a == c, "identical meshes are deduplicated");
}

static void test_determinism() {
    check(encode_scene(16) == encode_scene(16), "identical scenes produce identical bytes");
    const std::vector<uint8_t> buf = encode_scene(16);
    const size_t node_padding = sizeof(Header) + 4;
    check(std::memcmp(buf.data() + node_padding, "\0\0\0\0", 4) == 0, "node padding bytes are zero");
}

static void test_input_validation() {
    Encoder enc;
    std::vector<float> tri = {0, 0, 0, 1, 0, 0, 0, 1, 0}, n;
    check(enc.add_geometry(tri, n, {0, 1, 3}) == Encoder::INVALID_ID, "rejects index out of range");
    check(enc.add_geometry({0, 0, 0, 1}, n, {0, 0, 0}) == Encoder::INVALID_ID, "rejects vertex count not multiple of 3");
    check(enc.add_geometry(tri, n, {0, 1}) == Encoder::INVALID_ID, "rejects index count not multiple of 3");
    std::vector<float> nan_tri = tri;
    nan_tri[4] = std::numeric_limits<float>::quiet_NaN();
    check(enc.add_geometry(nan_tri, n, {0, 1, 2}) == Encoder::INVALID_ID, "rejects NaN coordinates");
    check(!enc.add_node(5, kIdentity), "rejects node with unknown geometry id");
}

static bool decodes(const std::vector<uint8_t>& buf) {
    Decoder dec;
    return dec.read_memory(reinterpret_cast<const char*>(buf.data()), buf.size());
}

static void test_corrupted_files() {
    const std::vector<uint8_t> good = encode_scene(16);
    check(decodes(good), "valid file decodes");

    // Every truncation must be rejected without crashing.
    bool all_rejected = true;
    for (size_t len = 0; len < good.size(); ++len) {
        std::vector<uint8_t> cut(good.begin(), good.begin() + len);
        if (decodes(cut)) all_rejected = false;
    }
    check(all_rejected, "every truncated file is rejected");

    // Corrupted header fields.
    auto patch32 = [&](size_t offset, uint32_t value) {
        std::vector<uint8_t> b = good;
        std::memcpy(b.data() + offset, &value, 4);
        return b;
    };
    auto patch64 = [&](size_t offset, uint64_t value) {
        std::vector<uint8_t> b = good;
        std::memcpy(b.data() + offset, &value, 8);
        return b;
    };
    const size_t off_num_geometries = offsetof(Header, num_geometries);
    const size_t off_num_nodes = offsetof(Header, num_nodes);
    std::vector<uint8_t> bad_magic = good;
    bad_magic[0] = 'X';
    check(!decodes(bad_magic), "rejects wrong magic");
    check(!decodes(patch32(offsetof(Header, version), 200)), "rejects old format version");
    check(!decodes(patch32(off_num_nodes, 4000000000u)), "rejects huge node count");
    check(!decodes(patch32(off_num_geometries, 4000000000u)), "rejects huge geometry count");
    check(!decodes(patch64(offsetof(Header, offset_hashtable), ~0ull)), "rejects node offset beyond the file");
    check(!decodes(patch64(offsetof(Header, offset_geometries), ~0ull)), "rejects geometry offset beyond the file");
    check(!decodes(patch32(sizeof(Header), 99)), "rejects node pointing to a missing geometry");

    // Random byte flips: may decode or not, but must never crash
    // (run with AddressSanitizer in CI to detect invalid memory access).
    std::mt19937 rng(12345);
    for (int trial = 0; trial < 2000; ++trial) {
        std::vector<uint8_t> b = good;
        const int flips = 1 + static_cast<int>(rng() % 4);
        for (int f = 0; f < flips; ++f) b[rng() % b.size()] ^= static_cast<uint8_t>(1u << (rng() % 8));
        decodes(b);
    }
    check(true, "2000 randomly corrupted files decoded without crashing");
}

static void test_c_api() {
    const std::vector<uint8_t> buf = encode_scene(12);
    v4s12_decoder_t* dec = v4s12_decoder_create();
    const int ok = v4s12_decoder_load_memory(dec, reinterpret_cast<const char*>(buf.data()), buf.size());
    uint32_t vcount = 0, icount = 0, geom_id = 99, color = 0;
    double transform[16];
    v4s12_decoder_get_geometry_info(dec, 0, &vcount, &icount);
    v4s12_decoder_read_node(dec, 0, &geom_id, transform, &color);
    v4s12_decoder_destroy(dec);
    check(ok == 1 && vcount > 0 && icount > 0 && geom_id == 0 && color == 0x11223344u, "C API decodes geometry and node");
}

int main() {
    test_round_trip(16);
    test_round_trip(12);
    test_round_trip(8);
    test_deduplication();
    test_determinism();
    test_input_validation();
    test_corrupted_files();
    test_c_api();

    std::printf(failures ? "\n%d test(s) FAILED\n" : "\nAll tests passed\n", failures);
    return failures ? 1 : 0;
}
