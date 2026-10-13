# SpaceAGORAMuJoCo.jl

Optional MuJoCo proximity scenes for [SpaceAGORA](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl).
This companion runs standalone scenes (Stage 1) or carries spacecraft inside
`run_simulation` through the core's external-propagation adapter contract (Stage 2).
Actuation and sensors inside coupled scenes remain Stage 3 work.

The core package does not depend on this repository. A plain `using SpaceAGORA`
does not load this adapter or download MuJoCo. Explicitly load both packages:

```julia
using SpaceAGORA, SpaceAGORAMuJoCo
EP = SpaceAGORA.ExternalPropagationServices

# Given an MJCF string and inertial initial position/velocity vectors:
scene = ProximityScene(; mjcf_xml=xml, dt=0.02,
    planet=SpaceAGORA.make_no_gram_planet(:earth),
    gravity_effectors=(SpaceAGORA.InverseSquaredGravityModel(),),
    initial_states=[SceneBodyState("chaser", r1, v1), SceneBodyState("target", r2, v2)])
scene_step!(scene)
scene_body_state(scene, "chaser")  # absolute inertial (r, v, q, ω)

# Given a compatible SpaceAGORA SimulationConfiguration:
config = EP.with_external_propagators(config,
    (ProximitySceneDynamics(scene, 1 => "chaser", 2 => "target"),))
SpaceAGORA.run_simulation(config)
```

See [architecture and conventions](docs/architecture.md) for frames, units,
ownership, solver restrictions and the measured first-order convergence.

## First release preparation

The [v0.1.0 draft release notes](docs/releases/v0.1.0.md) record exact qualified
core/companion revisions, a clean installation recipe, native test evidence and
current limits. This is preparation for a release, not an announcement of a
published tag or registry entry.

## Installation

Julia 1.12 is required. Native scenes currently support **Linux x86_64 only**.
Clone this repository and build a separate environment:

```sh
git clone https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl.git
cd SpaceAGORAMuJoCo.jl
julia --startup-file=no scripts/setup_env.jl /path/to/mujoco-env
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/mujoco-env
```

The setup uses command-line Git to check out the exact SpaceAGORA commit recorded
in `Project.toml` under the environment's `sources/SpaceAGORA-<SHA>` directory,
then develops that core checkout and this companion and declares the direct
imports needed by its tests and benchmark driver. It creates its own manifest;
it never copies a parent manifest or calls a HYPR installer. No native GRAM data
or parent repository checkout is needed. Git is required; submodules are not
initialized and LFS files remain pointers. The managed core checkout is verified
before package installation. Repeat setup reuses it only when its recorded
source, HEAD, package identity and clean state match; a modified or mismatched
checkout is refused without resetting it. Keep the environment and its `sources`
directory together while using the installation. Dependency installation and Julia
precompilation occur only when you run this setup, not when importing the adapter.
The lazy MuJoCo artifact downloads when a native operation first needs it; its
archive and unpacked tree are verified against `Artifacts.toml`.

The `SpaceAGORA = "0.2"` compatibility entry is an outer version-family constraint,
not a claim that every 0.2 release supports this adapter. The supported default
is the **exact core source pin plus `ExternalPropagationServices` contract 1.0.0**.
Source entries do not propagate into arbitrary consumer environments; use the
setup above or explicitly install that core revision before adding the companion.
The adapter checks the contract before defining hook methods and again at runtime
initialization, including when loading a precompiled package. The shared
[adapter contract standard](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/0a86898c52c16c09aa187c761cc5ef37082ee8a5/docs/src/maintainer/adapter_contract.md)
and [external propagation service reference](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/0a86898c52c16c09aa187c761cc5ef37082ee8a5/docs/src/maintainer/external_propagation_services.md)
describe the core-owned interface.

For integration development, override the core checkout or a full commit SHA:

```sh
SPACEAGORA_CORE_PATH=/path/to/SpaceAGORA.jl \
  julia --startup-file=no scripts/setup_env.jl /path/to/integration-env
# Alternatively, SPACEAGORA_CORE_REV=<full-40-character-SHA> selects another
# revision from the core repository. Set only one override.
```

Overrides are compatibility probes; they do not expand the supported release
range. Core integration CI uses `SPACEAGORA_CORE_PATH` with its exact checkout.

## Validation

Run the native suite with the library required and four threads, including the
threaded Monte Carlo identity checks:

```sh
SPACEAGORA_MUJOCO_REQUIRED=1 JULIA_NUM_THREADS=4 JULIA_LOAD_PATH='@:@stdlib' \
  julia --startup-file=no --project=/path/to/mujoco-env test/runtests.jl
julia --startup-file=no --project=/path/to/mujoco-env test/load_order.jl core-first
julia --startup-file=no --project=/path/to/mujoco-env test/load_order.jl companion-first
julia --startup-file=no --project=/path/to/mujoco-env test/core_only.jl
julia --startup-file=no --project=/path/to/mujoco-env test/incompatible_core.jl
```

The standalone CI installs from scratch and runs both import orders, core-only
isolation, early contract-mismatch refusal, contract conformance, and mandatory
native tests on Linux x86_64. The numerical tests,
model files, tolerances and pinned MuJoCo binding come from the original Stage 1/2
implementation. Extraction does not constitute a new scientific validation run.
Without `SPACEAGORA_MUJOCO_REQUIRED=1`, the native suite may skip on an unsupported
platform or unavailable artifact; such a skip is not native acceptance.

Bounded checks without a simulation or native library:

```sh
julia --startup-file=no test/installation_plan_tests.jl
julia --startup-file=no test/source_acquisition_tests.jl
python3 -B -m unittest discover -s benchmarks/basilisk_mujoco_xval -p test_harness.py
```

The Python guard tests require NumPy and Python 3.11 or later. The complete
[Basilisk comparison harness](benchmarks/basilisk_mujoco_xval/README.md), its cases
and [third-party notices](benchmarks/basilisk_mujoco_xval/NOTICE.md) live here.
The coupled timing study from PR253 remains in SpaceAGORA core.

## Provenance and license

This repository preserves the extracted package and cross-validation harness Git
histories from SpaceAGORA, including the reviewed Stage 1/2 implementation. The
package UUID and initial version are unchanged. Repository code is covered by
[LICENSE](LICENSE); the MuJoCo artifact carries Apache-2.0 notices, and the
Basilisk-derived MJCF models retain their ISC notice. The chief-frame convention
acknowledges the mjorbit project; no mjorbit source file is copied here.

## Dependency review

| | |
|---|---|
| Library | MuJoCo 3.11.0, official Google DeepMind release `mujoco-3.11.0-linux-x86_64.tar.gz` |
| License | Apache-2.0 (`LICENSE` and `THIRD_PARTY_NOTICES` ship in the tarball) |
| sha256 | `681a9353b296a06682e9e757d6d21ab7a0a5a5bdb218a1c05cb9a38536817c79` (equals the published `.sha256` file and the GitHub asset digest) |
| Size | 31.7 MB download, 85 MB unpacked, `libmujoco.so.3.11.0` 6.1 MB |
| Links against | libc, libm, libdl, libpthread, librt only |
| Platforms | Linux x86_64. Other platforms raise an unsupported-platform error: the Windows zip and macOS dmg were not verified or tested, and `MuJoCo_jll` was at 3.1.6 (2024) in the original dependency review |
| Why not `MuJoCo_jll` | The original review found only 2.3.7 and 3.1.6 in General, with the Yggdrasil recipe pinned to 3.1.6 |

MuJoCo runs in-process: a native fault ends the Julia process. `mjtBool` replaces `mjtByte` for
`eq_active` in 3.11.0 (still in `mjData`).
