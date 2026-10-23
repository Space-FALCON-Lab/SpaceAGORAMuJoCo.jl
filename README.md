# MuJoCo for SpaceAGORA

SpaceAGORAMuJoCo brings MuJoCo's rigid bodies, articulated joints and contacts
into orbital proximity scenes. Step a scene on its own, or let it propagate
selected spacecraft inside [SpaceAGORA](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl)'s
`run_simulation`. SpaceAGORA supplies the gravitational field; MuJoCo evolves
the mechanical scene. Installing or importing SpaceAGORA alone does not load
this companion.

Start with [installation](#installation), run the [first example](#first-example),
then read how the scene [connects to SpaceAGORA](#connection-to-spaceagora).
[Compatibility](#compatibility) and [validation limits](#validation-and-limits)
explain which configurations are supported.

The current release, [MuJoCo for SpaceAGORA v0.5.0](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/releases/tag/v0.5.0),
pairs this companion with SpaceAGORA 0.6.0 and `ExternalPropagationServices`
contract 2.0.0; its release page identifies the source pair. Releases from v0.2.0
on use that contract. The first release, v0.1.0, used contract 1.0.0 with its own
pinned core; its [release preparation record](docs/releases/v0.1.0.md) is retained
as historical documentation.

## Installation

Native scenes support **Linux x86_64** and require Julia 1.12 and Git. To install
the v0.5.0 release in a separate environment:

```sh
git clone https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl.git
cd SpaceAGORAMuJoCo.jl
git checkout --detach v0.5.0
unset SPACEAGORA_CORE_PATH SPACEAGORA_CORE_REV
julia --startup-file=no scripts/setup_env.jl /path/to/mujoco-env
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/mujoco-env
```

In Julia, load both packages:

```julia
using SpaceAGORA, SpaceAGORAMuJoCo
```

The installer checks out the exact SpaceAGORA revision recorded in
`SpaceAGORASource.toml` on the development branch (`Project.toml` on the
immutable v0.5.0 tag), develops that source and this companion, and creates the
environment's manifest. Keep the companion checkout, environment and its
`sources/` directory together: the environment refers to those local paths.
Repeat setup reuses the managed core checkout only when its source, revision,
package identity and clean state match. It refuses a modified or mismatched
checkout without resetting it.

Setup installs dependencies and precompiles Julia packages. The lazy MuJoCo
3.11.0 artifact downloads when a native operation first needs it; its archive
and unpacked tree are checked against `Artifacts.toml`.

Without the `git checkout` line, setup uses the default branch and the core
revision its `SpaceAGORASource.toml` records. The
[user guide](docs/user_guide.md#choose-the-guide-and-source-pair-together)
describes both routes; keep the guide, the companion and the core from the same
pair.

## First example

The [two-spacecraft example](docs/user_guide.md#a-complete-standalone-example)
in the user guide defines the complete MJCF model, initial states and gravity
settings. Save it as `two_cubes.jl` and run it in the environment above:

```sh
julia --startup-file=no --project=/path/to/mujoco-env two_cubes.jl
```

The example uses two non-contacting cubes so you can first check orbital motion
and state conventions. Continue with
[carrying the scene inside SpaceAGORA](docs/user_guide.md#carrying-the-scene-inside-spaceagora)
to build a full simulation configuration and read its saved table. The guide
also explains frames, quaternions, resetting scenes and applying standalone
forces.

## Connection to SpaceAGORA

The companion implements `SpaceAGORA.ExternalPropagationServices` contract
**2.0.0**, as every release from v0.2.0 on does; v0.1.0 implemented contract 1.0.0
with its own pinned core. `ProximitySceneDynamics` assigns the scene exclusive propagation
ownership of selected spacecraft. Each coupled run prepares its own native
scene from the template; other spacecraft may continue to use the core
propagator in the same run.

In a SpaceAGORA source checkout, MuJoCo is organized under
`packages/robotics/SpaceAGORAMuJoCo.jl`. To use that checkout's selected companion
revision, run this separate setup route from the **SpaceAGORA repository root**:

```sh
julia --startup-file=no scripts/optional_packages.jl setup mujoco /path/to/mujoco-env
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/mujoco-env
```

Then load `SpaceAGORA` and `SpaceAGORAMuJoCo`. This route uses the current core
checkout and its recorded companion revision; it does not change this
repository's standalone default core pin. See the
[Robotics package guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/main/packages/robotics/README.md)
for checkout management and the
[SpaceAGORA MuJoCo guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/main/docs/src/user/mujoco_scenes.md)
for the parent package's entry points.

MuJoCo contacts act on geometry inside the native scene. Contact-force exchange
with SpaceAGORA's **compliant multibody dynamics** is **not implemented or
validated**. That two-way coupling is tracked in
[issue #4](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/issues/4).
Core actuation and sensors are also not connected to coupled scenes. Read the
[scope and practical limits](docs/user_guide.md#scope-and-practical-limits)
before combining models.

## Compatibility

The `SpaceAGORA = "0.5, 0.6"` entry is a version-family constraint. SpaceAGORA
0.5.0 removes the extension-hook names deprecated in 0.3.0, which this package no
longer uses. SpaceAGORA 0.6.0 changes the HYPR and EDG service contracts and
keeps `ExternalPropagationServices` contract 2.0.0, so both families provide the
contract this package implements. The pinned core revision is SpaceAGORA 0.6.0,
recorded in `SpaceAGORASource.toml` on this branch (unchanged from the v0.5.0
release pair). The supported
standalone default is the **exact core source pin plus
`ExternalPropagationServices` contract 2.0.0**; the version families alone do
not qualify another source revision.
This separate pin file is consumed by the installer and contributor workflow;
Pkg does not read it when adding the companion to another environment. Use the
installer or explicitly select that core revision before adding the companion.

The adapter checks the contract before defining hook methods and again at
runtime initialization, including when loading a precompiled package. The
[adapter standard](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/0487e8226107592b19221618f0f21f500cf2aacf/docs/src/reference/adapter_contract.md)
and [external propagation reference](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/0487e8226107592b19221618f0f21f500cf2aacf/docs/src/reference/external_propagation_services.md)
describe the core-owned interface. The [architecture note](docs/architecture.md)
explains scene ownership and synchronization.

For integration development, override the core checkout or a full commit SHA:

```sh
SPACEAGORA_CORE_PATH=/path/to/SpaceAGORA.jl \
  julia --startup-file=no scripts/setup_env.jl /path/to/integration-env
# Alternatively, SPACEAGORA_CORE_REV=<full-40-character-SHA> selects another
# revision from the core repository. Set only one override.
```

Overrides are compatibility probes; they do not expand the supported release
range. Core integration CI uses `SPACEAGORA_CORE_PATH` with its exact checkout.

<a id="validation"></a>

## Validation and limits

Run the native suite with the library required and four threads, including the
threaded Monte Carlo identity checks:

```sh
SPACEAGORA_MUJOCO_REQUIRED=1 JULIA_NUM_THREADS=4 JULIA_LOAD_PATH='@:@stdlib' \
  julia --startup-file=no --project=/path/to/mujoco-env test/runtests.jl
julia --startup-file=no --project=/path/to/mujoco-env test/load_order.jl core-first
julia --startup-file=no --project=/path/to/mujoco-env test/load_order.jl companion-first
julia --startup-file=no --project=/path/to/mujoco-env test/core_only.jl
julia --startup-file=no --project=/path/to/mujoco-env test/incompatible_core.jl
julia --startup-file=no --project=/path/to/mujoco-env test/missing_native.jl
```

For the default pair, CI calls SpaceAGORA's reusable contributor workflow at the
same exact revision as `SpaceAGORASource.toml`. It builds a separate environment
seeded from the core manifest, precompiles strictly, and runs the suite, both
load orders and incompatible-core refusal with deprecations treated as errors.
Its direct test runner does not add Pkg.test's forced bounds-checking flag.

The required independent installation job runs installer/source-acquisition
checks, core-only isolation, both load orders, incompatible-core and missing-library
refusal, and mandatory native tests with four threads and
`SPACEAGORA_MUJOCO_REQUIRED=1` on every push and pull request. The Python benchmark
guards and release-pin job remain separate. A manually requested `core_revision`
skips the fixed-pair reusable call; the installer job runs the same checks against
that override.

The standalone CI installs from scratch and runs both import orders, core-only
isolation, early contract-mismatch refusal, contract conformance, the refusal of a
scene without the MuJoCo library, and mandatory native tests on Linux x86_64. Both
import orders also check that the package uses only names the contract lists and
SpaceAGORA's public root names, also through local aliases and in function bodies. A separate job checks that the pinned core commit is
a SpaceAGORA release. The numerical tests,
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

The Python guard tests require NumPy and Python 3.11 or later. The
[Basilisk comparison](benchmarks/basilisk_mujoco_xval/README.md) runs MuJoCo on
both sides. It checks the surrounding frame, gravity, actuation and integration
choices, but agreement can miss an error shared by the native engine. Its
contact case remains **inconclusive**. The guide records convergence evidence
and limitations; each new scene needs its own model and resolution checks.
The models retain their [third-party notices](benchmarks/basilisk_mujoco_xval/NOTICE.md).
The coupled timing study from PR253 remains in SpaceAGORA core.

## Credits and license

**Author and project lead: Evan Yu.** This repository also preserves the work of
the SpaceAGORA contributors and the third-party acknowledgments below.

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
