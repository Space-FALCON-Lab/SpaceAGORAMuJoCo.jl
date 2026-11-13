# Unreleased: macOS on Apple Silicon

Native scenes now run on macOS arm64 as well as Linux x86_64, with the same
MuJoCo 3.11.0.

- **macOS arm64 added.** `scripts/setup_env.jl` downloads the official
  `mujoco-3.11.0-macos-universal2.dmg`, checks its sha256, and builds a Julia
  artifact from it, whose tree hash is pinned in `Artifacts.toml`. See
  [Running on macOS](../../README.md#running-on-macos-apple-silicon).
- **Linux x86_64 unchanged.** The artifact entry, download, checksums, code path
  and error messages are the same, and setup still downloads nothing native
  unless the new `--prefetch-native` flag is given.
- **Offsets verified identical.** Every `offsetof`, `sizeof` and enum value the
  binding uses is the same for the 3.11.0 headers on Linux x86_64 and macOS
  arm64, so `OFFSETS` did not change. On Apple Silicon the native suite's
  readback checks of every offset pass.
- **New files.** `NativeLibrary.toml` names each platform's source,
  `src/macos_artifact.jl` builds the macOS artifact, and
  `test/native_library_tests.jl` checks that `NativeLibrary.toml` and
  `Artifacts.toml` agree.
- **New standard-library dependencies.** `Downloads`, `Pkg`, `SHA` and `TOML`.
- **Still unsupported.** Intel Macs, Linux aarch64 and Windows raise the
  unsupported-platform error, whose text now names both supported platforms.
- **Known difference on macOS.** Two allocation-bound checks in
  `test/scene_tests.jl` are skipped, because they read glibc's `mallinfo2`.
