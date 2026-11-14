# Experimental bus–bumper engine coupling

`BusBumperContact` implements the new `SpaceAGORA.ContactLoadServices v0.1.0`
interface. Core owns both bodies and integration. The provider resets MuJoCo at
every trial state, supplies physical inertias and smooth COM loads, calls
`mj_forward`, checks the contact optimality residual, and returns contact-only
COM wrenches. It never calls a native time step.

The default released core pin is unchanged and does not provide this interface.
Existing proximity scenes continue to work there; constructing `BusBumperContact`
throws an actionable error. To reproduce the experimental pairing used by CI:

```sh
SPACEAGORA_CORE_REV=e5ccd9856ba0b5315aac3e5e7e9e1c790852d082 \
  julia --startup-file=no scripts/setup_env.jl /tmp/contact-env
julia --startup-file=no --project=/tmp/contact-env test/runtests.jl
julia --startup-file=no --project=/tmp/contact-env examples/compliant_contact.jl /tmp/contact-engine.toml
```

The provider supports one box centered on the rigid bus COM and one sphere
centered on its sole compliant attachment COM. Geometry is explicitly configured
in `BusBumperContact`; the example matches it to the physical bus and bumper.
Mass and full body-frame COM inertia come from core. Principal inertias and an
explicit high-accuracy inertial-frame quaternion avoid a measurable XML
`fullinertia` frame-conversion error. Both free-joint frames remain unaligned.

The core refuses unsupported combinations before native allocation. Start with
serial Tsit5, no gravity or other effectors/devices, one isotropic joint, and
both maximum step caps `2^-12` seconds. The native timestep stays fixed during
core-step refinement. The provider uses Newton, dense constraints, an elliptic
cone, disabled warm starts/sleep, exact diagonal contact inertia and an explicit
pair. Normal-only and isotropic sliding friction are supported by this bounded
experiment; this is not a general collision-geometry interface.

The independent contact check uses the physical inverse mass matrix and a
projected-gradient optimality residual in force units. Each query fails for
warnings, unexpected contact/constraint counts, nonfinite values or a residual
above `1e-10 N`; it never returns a stale force. Runtime cleanup is idempotent.

## Evidence and remaining qualification

Local clean-process tests cover load projection, A–B–A and reversed-time purity,
separation, independent runtimes, full-inertia spinning free acceleration,
head-on approach/recoil/separation, off-center frictional rotation, momentum and
angular momentum, translated/boosted frames, scalar-reference agreement and four
core step caps. The head-on work calculation independently integrates contact
power, splitting the first-touch discontinuity before Gauss quadrature. Its
quadrature is refined separately from the trajectory. No tolerance was relaxed.

The reproducible two-second example writes penetration, peak load, contact
duration, impulse, final relative state, momentum error and energy-work residual
for all four step caps. The reported impulse and work use the independently
implemented scalar law for this head-on fixture; they are not general-purpose
provider telemetry. Force signs and the scalar law are separately checked
against native constraints in the feasibility tests. CI uploads one report per
platform. Tight adaptive tolerances can put all four caps near the same error
floor; do not infer an event convergence order from a cap sweep alone.

This continues [Issue #4](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/issues/4)
and does not close it. Remaining work includes forced solver-rejection replay,
full rotated-trajectory comparisons, grazing/stick–slip transition ledgers,
independent tolerance/native-regularization sweeps and complete machine-readable
trajectory/energy ledgers for the off-center cases. The
[architecture and validation plan](design/compliant_contact_coupling.md) remains
the acceptance matrix for broader support.
