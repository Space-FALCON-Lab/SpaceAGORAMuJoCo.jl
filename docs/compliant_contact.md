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
SPACEAGORA_CORE_REV=9ee7817763fc5a8d4d5438ca2235393e618e8d7d \
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
pair. Normal-only and isotropic sliding friction are implemented in this bounded
experiment; physical qualification of friction is still incomplete. This is not
a general collision-geometry interface.

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

The [expanded validation report](design/contact_validation_results.md) adds
forced-rejection replay, rotated trajectories, grazing and low-slip ledgers,
independent core/native parameter sweeps and full machine-readable trajectories.
It records two friction qualification counterexamples and a default-tolerance
native refusal with a stricter-solve recovery. Issue #4 remains open; these
results do not qualify a general stick/slip model. The
[architecture and validation plan](design/compliant_contact_coupling.md) remains
the acceptance matrix for broader support.

## Friction qualification blocker

Additional Issue #4 trajectory validation found positive tangential power in
an off-center sliding case (mount offset 0.2 m, initial tangential speed 0.4 m/s,
friction coefficient 0.1). At 0.052734375 s, the measured tangential power was
about +0.013129 W with nonzero slip and a force on the friction-cone boundary.
Core step caps from `2^-10` to `2^-14` s with tolerance `1e-12` reproduced it;
world-wrench power and native constraint power agreed within `3e-15 W`.
A low-slip case (0.01 m/s, coefficient 0.8) also produced positive net tangential
work. Total mechanical energy still decreased in both cases.

These observations fail the planned opposing-slip-power qualification criterion.
They are a limitation of the currently queried native soft-contact response,
not evidence that the friction model is physically qualified. MuJoCo documents
that its [convex soft-contact model](https://mujoco.readthedocs.io/en/3.11.0/computation/index.html#physical-realism-and-soft-contacts)
need not have maximally dissipative friction; that does not waive our acceptance
target. Keep friction experimental, retain this counterexample in the validation
report, and leave Issue #4 open pending a justified model/scope decision. Do not
clip returned forces, relax the power criterion, or mistake a closed energy-work
ledger for proof of dissipative friction.
