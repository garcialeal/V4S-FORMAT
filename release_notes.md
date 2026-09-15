# Release v1.0.0 — WebAssembly & Native Windows CLI Distribution

First official stable release of the **v4s Format Specification** and the **V4S12-MATHEMATICAL-CORE** pipeline.

### 🚀 What's Included in this Release

* **WebAssembly SDK (`v4s12-wasm-v1.0.0.zip`)**:
  * `v4s12_decoder.wasm` & `v4s12_decoder.js` — Client-side web runtime for fast 3D mesh decompression.
  * `v4s12_encoder.wasm` & `v4s12_encoder.js` — Client-side binary serialization suite.

* **Windows Native CLI (`v4s12-cli-v1.0.0-win64.zip`)**:
  * `v4s_info.exe` — Binary mesh inspector and metadata extractor.
  * `generate_benchmarks.exe` — Performance testing & compression ratio validation suite.
  * `test_v2.exe` — Verification suite for format integrity.

### 🔑 Key Features & Architecture
* **Decoupled Engine Submodules**: Core algebra and fast spatial hashing decoupled into `external/V4S12-MATHEMATICAL-CORE` and `external/HASH-G3-RH512-256`.
* **V4 Spectral Transform & $S_{12}$ Lattice**: Deterministic bitwise 0-FLOP geometry processing ($\Delta_{max} \approx 0.0004884$).
* **ZSTD Compressed Payload**: Integrated stream-level compression.
* **C-API Interoperability**: Clean C-interface ready for WASM, Web Workers, Three.js, WebGPU, C#, and Python embeddings.
