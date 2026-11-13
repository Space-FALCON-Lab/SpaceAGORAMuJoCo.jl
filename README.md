# MuJoCo for SpaceAGORA

SpaceAGORAMuJoCo brings MuJoCo's rigid bodies, articulated joints and contacts
into orbital proximity scenes. You can step a scene on its own, or let it
propagate selected spacecraft inside [SpaceAGORA](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl)'s
`run_simulation`. SpaceAGORA supplies the gravitational field; MuJoCo evolves
the mechanical scene. Installing or loading SpaceAGORA alone does not load this
companion.

Start with [installation](#installation), run the [first example](#first-example),
then read how the scene [connects to SpaceAGORA](#connection-to-spaceagora).
[Compatibility](#compatibility) and [validation and limits](#validation-and-limits)
explain which configurations are supported.

**Supported platforms:** Linux x86_64, and macOS on Apple Silicon (arm64). Intel
Macs, Linux on ARM and Windows are not supported: packages install there, but a
native scene raises an unsupported-platform error.

The latest release, [MuJoCo for SpaceAGORA v0.5.0](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/releases/tag/v0.5.0),
pairs this companion with SpaceAGORA 0.6.0; its release page names the source
pair. The current `main` also admits SpaceAGORA 0.7 and pins SpaceAGORA v0.7.0.
Every release from v0.2.0 on implements `ExternalPropagationServices` contract
2.0.0. The first release, v0.1.0, used contract 1.0.0 with its own pinned core;
its [release preparation record](docs/releases/v0.1.0.md) is kept as history.

## Installation

Native scenes run on Linux x86_64 and on macOS with Apple Silicon, and need
Julia 1.12 and Git. The commands are the same on both; macOS has
[its own notes](#running-on-macos-apple-silicon). To install the
current checkout in a separate environment:

```sh
git clone https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl.git
cd SpaceAGORAMuJoCo.jl
unset SPACEAGORA_CORE_PATH SPACEAGORA_CORE_REV
julia --startup-file=no scripts/setup_env.jl /path/to/mujoco-env
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/mujoco-env
```

In Julia, load both packages:

```julia
using SpaceAGORA, SpaceAGORAMuJoCo
```

The installer checks out the exact SpaceAGORA revision recorded in
`SpaceAGORASource.toml` (SpaceAGORA v0.7.0 on `main`), develops that source and
this companion, and creates the environment's manifest. Keep the companion
checkout, the environment and its `sources/` directory together: the environment
refers to those local paths. Running setup again reuses the managed core checkout
only if its source, revision, package identity and clean state match; it refuses
a modified or mismatched checkout without resetting it.

The installer also writes an ownership record, `.spaceagora-mujoco-environment.toml`,
for the companion and core pair, and reuses an environment only when that record
matches. Environments made by earlier installers do not have the record: create a
new, empty environment outside both checkouts and keep the old one. Setup refuses
to take over a nonempty directory it did not create.

Setup installs the dependencies and precompiles the Julia packages. The MuJoCo
3.11.0 artifact is downloaded the first time a native operation needs it, and
both the archive and the unpacked tree are checked against `Artifacts.toml`.

`Artifacts.toml` and `NativeLibrary.toml` select the native library for the
host's operating system and architecture. Linux downloads it on first native
use, or at setup if you add `--prefetch-native` after the environment directory.
macOS downloads it at setup.

To use SpaceAGORA v0.7.1, either install from a SpaceAGORA v0.7.1 checkout as
described in [Connection to SpaceAGORA](#connection-to-spaceagora), or point this
installer at the release commit with
`SPACEAGORA_CORE_REV=db33b871795c9777037e6b6538996b08faec9aff` (see
[Compatibility](#compatibility)). The `v0.5.0` tag cannot be used with SpaceAGORA
0.7: it admits SpaceAGORA 0.5 and 0.6 and installs v0.6.0. The
[user guide](docs/user_guide.md#choose-the-guide-and-source-pair-together)
describes these routes; keep the guide, the companion and the core from the same
pair.

### Running on macOS (Apple Silicon)

You need an Apple Silicon Mac, Julia 1.12 (for example `juliaup add 1.12`), and
Git from the Xcode Command Line Tools (`xcode-select --install`). `hdiutil`, which
setup uses, is part of macOS.

The quickest route is the guided script, which checks the Mac, offers to install
Julia, runs the installer below and confirms that MuJoCo loads. A second script
then runs every check in [Validation and limits](#validation-and-limits) and
prints a summary:

```sh
test/support/macos_first_light.sh      # first-time setup; asks before installing anything
test/support/macos_shakedown.sh        # tests; add --full to also rebuild MuJoCo from scratch
```

Both accept `--env DIR`, `--yes` and `--help`. To install by hand instead, use the
same commands as on Linux:

```sh
git clone https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl.git
cd SpaceAGORAMuJoCo.jl
unset SPACEAGORA_CORE_PATH SPACEAGORA_CORE_REV
julia --startup-file=no scripts/setup_env.jl /path/to/mujoco-env
```

On macOS, setup has one more step after the packages are installed. Google
DeepMind publishes the macOS build as a disk image, which Julia's artifact
system cannot unpack, so the installer builds the artifact itself:

1. It finds the `macos_aarch64` table in `NativeLibrary.toml`.
2. It downloads the official `mujoco-3.11.0-macos-universal2.dmg` (about 46 MB)
   and checks its sha256 against the value in that table.
3. It mounts the image read-only, copies `mujoco.framework`, `LICENSE` and
   `THIRD_PARTY_NOTICES` into a Julia artifact, and detaches the image.
4. It accepts the artifact only if its tree hash equals the one pinned in
   `Artifacts.toml`, then loads the library and prints
   `Native library: MuJoCo 3011000 is installed and loads`.

Later runs reuse the artifact and download nothing. Start Julia and check that
the native library loads:

```sh
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/mujoco-env
```

```julia
using SpaceAGORA, SpaceAGORAMuJoCo
SpaceAGORAMuJoCo.Binding.version() == 3011000
```

The [first example](#first-example), the user guide's two-cube example and the
commands under [Validation and limits](#validation-and-limits) run unchanged. Two
memory checks in the native suite report "broken" on macOS: their bound is read
from glibc's `mallinfo2`, which macOS does not have.

If something goes wrong:

| Symptom | What to do |
|---|---|
| `MuJoCo disk image checksum mismatch` | The download was incomplete or altered, and nothing was installed. Run setup again. If it repeats, compare the file at the URL in `NativeLibrary.toml` with the sha256 there; do not edit the pinned value. |
| A MuJoCo image is still mounted after an interrupted setup | `hdiutil info` lists attached images and their mount points; `hdiutil detach <mount point>` releases one. |
| `unsupported platform Darwin/x86_64` | Julia is an Intel build, either on an Intel Mac or under Rosetta. Install the native `aarch64` Julia on Apple Silicon; Intel Macs are not supported. |
| `libmujoco.3.11.0.dylib not found under the artifact` | The artifact directory exists but is incomplete. Remove the directory the message names and run setup again. |
| You want to rebuild the artifact | Remove `~/.julia/artifacts/bc122c81694ff4f9854d5c6168652408faf07262` and run setup again; it rebuilds a missing artifact. On Linux, add `--prefetch-native` to download at setup instead of on first use. |

## First example

The [two-spacecraft example](docs/user_guide.md#a-complete-standalone-example)
in the user guide defines the complete MJCF model, initial states and gravity
settings. Save it as `two_cubes.jl` and run it in the environment above:

```sh
julia --startup-file=no --project=/path/to/mujoco-env two_cubes.jl
```

The two cubes do not touch, so you can first check the orbital motion and the
state conventions. Continue with
[carrying the scene inside SpaceAGORA](docs/user_guide.md#carrying-the-scene-inside-spaceagora)
to build a full simulation configuration and read its saved table. The guide also
explains frames, quaternions, resetting scenes and applying forces to a
standalone scene.

## Connection to SpaceAGORA

The companion implements `SpaceAGORA.ExternalPropagationServices` contract 2.0.0.
`ProximitySceneDynamics` gives the scene sole ownership of the propagation of
selected spacecraft. Each coupled run prepares its own native scene from the
template, and other spacecraft in the same run can still use the core propagator.

A SpaceAGORA source checkout keeps this companion as a submodule at
`packages/robotics/SpaceAGORAMuJoCo.jl`; SpaceAGORA v0.7.1 records commit
`7b5a0031`, which has the same `src/` as the current `main`. To install that
recorded revision, run this from the root of a SpaceAGORA Git clone:

```sh
julia --startup-file=no scripts/optional_packages.jl setup mujoco /path/to/mujoco-env
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/mujoco-env
```

Then load `SpaceAGORA` and `SpaceAGORAMuJoCo`. This route uses your SpaceAGORA
checkout and the companion revision it records; it does not change this
repository's own default core pin. See SpaceAGORA's
[robotics package guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/packages/robotics/README.md)
for managing the checkout and the
[SpaceAGORA MuJoCo guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/docs/src/user/mujoco_scenes.md)
for SpaceAGORA's side.

MuJoCo contacts act on the geometry inside the native scene. Exchanging contact
forces with SpaceAGORA's compliant multibody dynamics is not implemented or
validated; that two-way coupling is tracked in
[issue #4](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/issues/4).
Core actuators and sensors are not connected to coupled scenes either. Read the
[scope and practical limits](docs/user_guide.md#scope-and-practical-limits)
before combining models.

## Compatibility

`Project.toml` on `main` admits SpaceAGORA 0.5, 0.6 and 0.7; the `v0.5.0` tag
admits 0.5 and 0.6. SpaceAGORA 0.5.0 removed the extension-hook names deprecated
in 0.3.0, which this package no longer uses. SpaceAGORA 0.6.0 changed the HYPR and
EDG service contracts but kept `ExternalPropagationServices` contract 2.0.0, and
0.7 keeps it too, so all three series provide the contract this package
implements. The supported default is the exact core revision in
`SpaceAGORASource.toml` (v0.7.0) with contract 2.0.0; the version range alone
does not qualify another core revision.

Pkg does not read `SpaceAGORASource.toml` when you add the companion to another
environment; only the installer and the contributor workflow use it. Use the
installer, or select that core revision yourself before adding the companion.

The adapter checks the contract version before defining its hook methods and
again when the package initializes, including when it is loaded from a
precompiled image. SpaceAGORA's
[adapter standard](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/bec28fcc348b48c6e807d007f9a5009e253e8340/docs/src/reference/adapter_contract.md)
and [external propagation reference](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/bec28fcc348b48c6e807d007f9a5009e253e8340/docs/src/reference/external_propagation_services.md)
describe the interface the core owns; these links point at the pinned v0.7.0
commit, and the two pages are unchanged in v0.7.1. The
[architecture note](docs/architecture.md) explains scene ownership and
synchronization.

For integration work, the installer accepts either a core checkout or a full
commit SHA:

```sh
SPACEAGORA_CORE_PATH=/path/to/SpaceAGORA.jl \
  julia --startup-file=no scripts/setup_env.jl /path/to/integration-env
# Or: SPACEAGORA_CORE_REV=<full 40-character SHA> selects another revision
# from the core repository. Set only one of the two.
```

An override is a compatibility probe: it does not widen the supported pair.
SpaceAGORA's own CI uses `SPACEAGORA_CORE_PATH` to test the companion revision its
checkout records against that exact checkout.

Linux x86_64 behaves as before the macOS support was added: the same artifact
entry, download, checksums, code path and error messages, and no native download
at setup unless `--prefetch-native` is given.

<a id="validation"></a>

## Validation and limits

Run the native suite with the library required and four threads, including the
threaded Monte Carlo identity checks, then the load-order and refusal checks:

```sh
export JULIA_LOAD_PATH='@:@stdlib'
SPACEAGORA_MUJOCO_REQUIRED=1 JULIA_NUM_THREADS=4 \
  julia --startup-file=no --project=/path/to/mujoco-env test/runtests.jl
julia --startup-file=no --project=/path/to/mujoco-env test/load_order.jl core-first
julia --startup-file=no --project=/path/to/mujoco-env test/load_order.jl companion-first
julia --startup-file=no --project=/path/to/mujoco-env test/core_only.jl
julia --startup-file=no --project=/path/to/mujoco-env test/incompatible_core.jl
julia --startup-file=no --project=/path/to/mujoco-env test/missing_native.jl
```

Without `SPACEAGORA_MUJOCO_REQUIRED=1`, the native suite may skip on an
unsupported platform or when the artifact is unavailable, and a skip is not a
native pass.

The main workflow, `.github/workflows/ci.yml`, runs on every push and pull
request:

| Job | Required | What it runs |
|---|---|---|
| `installation-and-native-tests` | yes | The installer and source-acquisition tests, a fresh installation with `scripts/setup_env.jl`, core-only isolation, the incompatible-core refusal, both load orders (which also check that the package uses only the names the contract lists and SpaceAGORA's public names, including through local aliases and inside function bodies), the refusal of a scene without the MuJoCo library, and the native suite on Linux x86_64 with four threads and `SPACEAGORA_MUJOCO_REQUIRED=1`. |
| `benchmark-harness-guards` | yes | The Python guard tests of the Basilisk comparison harness. |
| `contributor-contract` | no | SpaceAGORA's reusable contributor workflow at the revision in `SpaceAGORASource.toml`: strict precompilation, the suite run directly from `test/runtests.jl` with deprecations as errors (without `Pkg.test`'s bounds-checking flag), the incompatible-core refusal and both load orders, in an environment seeded from the core's manifest. It does not set `SPACEAGORA_MUJOCO_REQUIRED`. |
| `macos-arm64-native-tests` | no | The same steps as `installation-and-native-tests` on an Apple Silicon runner: the installer builds the MuJoCo artifact from the official disk image, then the isolation, refusal and load-order checks and the native suite run with four threads and `SPACEAGORA_MUJOCO_REQUIRED=1`. It also runs `test/native_library_tests.jl` and checks that no disk image is left mounted. |
| `core-release-pin` | no | Checks that the pinned core commit is a SpaceAGORA release tag and that `[compat]` admits it. |

A manual run with a `core_revision` input skips the contributor job, and the
installation job runs the same checks against that revision. A second
workflow, Shared companion support, runs on pull requests and pushes to `main` and
compares the generated support files with their source; see [Updating shared support](docs/architecture.md#updating-shared-support).

The numerical tests, model files, tolerances and pinned MuJoCo binding come from
the original Stage 1 and 2 implementation in SpaceAGORA; moving them to this
repository was not a new scientific validation.

Checks that need neither a simulation nor the native library:

```sh
julia --startup-file=no test/installation_plan_tests.jl
julia --startup-file=no test/source_acquisition_tests.jl
julia --startup-file=no test/native_library_tests.jl
python3 -B -m unittest discover -s benchmarks/basilisk_mujoco_xval -p test_harness.py
```

The Python guard tests need NumPy and either Python 3.11 or later or the `tomli`
package. The [Basilisk comparison](benchmarks/basilisk_mujoco_xval/README.md) runs
MuJoCo on both sides. It checks the surrounding frame, gravity, actuation and
integration choices, but agreement can miss an error in the native engine shared
by both. Its contact case remains inconclusive. The comparison records its
convergence evidence and limitations; each new scene needs its own model and
resolution checks. The models keep their
[third-party notices](benchmarks/basilisk_mujoco_xval/NOTICE.md).

## Credits and license

Author and project lead: Evan Yu. This repository also preserves the work of the
SpaceAGORA contributors and the third-party acknowledgments below.

The repository keeps the Git history of the package and of the cross-validation
harness from SpaceAGORA, including the Stage 1 and 2 implementation. The package
UUID and initial version are unchanged. Repository code is covered by
[LICENSE](LICENSE); the MuJoCo artifact carries Apache-2.0 notices, and the
Basilisk-derived MJCF models keep their ISC notice. The chief-frame convention
acknowledges the mjorbit project; no mjorbit source file is copied here.

## Dependency review

| | |
|---|---|
| Library | MuJoCo 3.11.0, official Google DeepMind release `mujoco-3.11.0-linux-x86_64.tar.gz` |
| License | Apache-2.0 (`LICENSE` and `THIRD_PARTY_NOTICES` ship in the tarball) |
| sha256 | `681a9353b296a06682e9e757d6d21ab7a0a5a5bdb218a1c05cb9a38536817c79` (equals the published `.sha256` file and the GitHub asset digest) |
| Size | 31.7 MB download, 85 MB unpacked, `libmujoco.so.3.11.0` 6.1 MB |
| Links against | libc, libm, libdl, libpthread, librt only |
| macOS library | The same release's `mujoco-3.11.0-macos-universal2.dmg`; the package copies `mujoco.framework`, `LICENSE` and `THIRD_PARTY_NOTICES` out of it |
| macOS sha256 | `54933aa1a22a212134eaa663e83f4e81d7d2f14d0a778d0ba4fa14f66633b4f1` (equals the GitHub asset digest); the built artifact's tree hash is pinned in `Artifacts.toml` |
| macOS size | 46.2 MB download, 14 MB artifact, `libmujoco.3.11.0.dylib` 9.7 MB (universal: x86_64 and arm64) |
| macOS links against | libSystem and libc++ only |
| Platforms | Linux x86_64 and macOS arm64. The struct offsets the binding uses are the same on both. Other platforms raise an unsupported-platform error: the Windows zip, Linux aarch64 tarball and the Intel slice of the macOS library were not verified or tested, and `MuJoCo_jll` was at 3.1.6 (2024) in the original dependency review |
| Why not `MuJoCo_jll` | The original review found only 2.3.7 and 3.1.6 in General, with the Yggdrasil recipe pinned to 3.1.6 |

MuJoCo runs in the Julia process, so a native fault ends the process. In 3.11.0,
`eq_active` in `mjData` has type `mjtBool` instead of `mjtByte`.

## Contributors

- [Giusy Falcone](https://github.com/gfalcon2)
- [Evan Yu](https://github.com/EvanY14)
