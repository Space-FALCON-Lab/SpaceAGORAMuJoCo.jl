# Experimental compliant contact

`SpaceAGORA.ContactLoadServices v0.2.0` keeps every bus, attachment and compliant
arm state in core. MuJoCo supplies collision geometry, contact Jacobians and
smooth acceleration. The companion solves contact loads and returns balanced
COM wrenches; it never takes a native time step. Core applies each wrench once,
including the physical bus reaction, after assembling all smooth joint loads.

The released-core installation pin remains unchanged. These experiments require
this exact development pair, also used by the mandatory experimental CI jobs:

```sh
SPACEAGORA_CORE_REV=50ba35f6174db59251b5d0e05bc987104f900beb \
  julia --startup-file=no scripts/setup_env.jl /tmp/contact-env
julia --startup-file=no --project=/tmp/contact-env test/runtests.jl
julia --startup-file=no --project=/tmp/contact-env examples/validate_contact.jl /tmp/contact-baseline --require-qualified
julia --startup-file=no --project=/tmp/contact-env examples/validate_expanded_contact.jl /tmp/contact-expanded --require-qualified
CONTACT_VALIDATION_DURATION_S=2 julia --startup-file=no --project=/tmp/contact-env examples/validate_expanded_contact.jl /tmp/contact-expanded-long --require-qualified
```

## Models and scope

| Provider | Geometry and bodies | Tangential law |
|---|---|---|
| `BusBumperContact` | One bus box and one attachment sphere, centered on their COMs | Passive regularized Coulomb; bounded creep |
| `PrimitiveContact` | Explicit sphere/box/capsule pairs on physical attachment or arm bodies; at most eight contact points | Passive regularized Coulomb; coupled normal complementarity |
| `CoulombContact` | One isolated sphere/sphere, box-face/sphere or box-face/capsule-end contact | Zero-slip Coulomb sticking, sliding, release and re-sticking |

`ContactGeometry(body, kind, size; position, quaternion)` uses a one-based body
slot: bus first, then attachment or arm state columns. Sizes are sphere radius,
box half-extents, or capsule radius and half cylindrical length. Pose is local
to the body's COM; the quaternion is scalar-last. Geometry contributes no extra
mass. Core supplies each body's physical mass and full inertia.

Use serial Tsit5, attitude integration, a fixed-duration gravity-free mission,
and finite nonnegative isotropic joint coefficients. Maximum core steps must
be at most `2^-10 s`; start at `2^-12 s` and refine. The existing compliant arm
uses one `RobotArmReactionEffector` with a fixed rest plan. Variable rest plans,
actuators, articulated backbones, mixed arm/attachment routes, external-owned
bodies, GNC, checkpoint/resume and custom callbacks remain refused.

The regularized multiple-contact solver requires a monotone normal response.
The exact model requires a nonsingular, non-self-locking response and a qualified
moving-contact Jacobian derivative. Exact simultaneous sticking, box-edge/corner
sticking, capsule side manifolds, arbitrary meshes and external targets are not
supported. Unsupported contact states fail explicitly. The additional primitive
and simultaneous-contact coverage uses `PrimitiveContact`; it does not imply
that every primitive pair has an exact-sticking implementation.

## Regularized friction

`BusBumperContact(friction=0.4, slip_speed=0.001)` and `PrimitiveContact` use
Coulomb-magnitude friction above 1 mm/s and a passive viscous approximation below
that speed. Friction power is nonpositive. The default permits up to 1 mm/s creep,
with a 2 mm bound over two seconds entirely in that regime. Measure actual speed,
duration and displacement for each fixture. `slip_speed` changes the physical
approximation; reducing it does not implement exact sticking.

The [passive law and independent reference benchmarks](design/passive_friction.md)
and [merged baseline qualification](design/contact_validation_results.md) retain
the original counterexamples and budgets. The former native soft-friction law
added about 0.013129 W along slip and 2.32e-6 J in its low-slip counterexample.
Those failures are retained as regressions, not waived.

## Exact Coulomb sticking

`CoulombContact(geometries, pairs; friction=0.4, capture_speed=1e-7)` supports a
subcritical tangential load at zero slip. Normal contact remains compliant; this
is exact tangential sticking, not rigid nonpenetration. With contact Jacobian
`J`, physical inverse mass `M⁻¹`, and `W = J M⁻¹ J'`, sticking solves
`(W + diag(Rn,0,0)) f = -b`. The normal component of `b` includes the soft normal
reference acceleration. Tangential components include the moving-contact term
`J̇ v`, differentiated at the actual collision point. Accept the mode only when
normal force is nonnegative and tangential force lies inside the Coulomb disk.
Sliding force lies on that disk and opposes slip. Breakaway at zero slip solves
for a consistent acceleration direction, including normal/tangential coupling.

Trials read the committed mode without changing it. Core localizes release and
capture events and commits changes only at initialization, events or accepted
steps. Capture applies a mass-weighted tangential velocity projection. Core
checks total linear/angular impulse and kinetic-energy change, and records
projection work separately. `capture_speed` is a numerical event tolerance,
not a creep parameter. Qualification independently refines it, integration
steps and integration tolerances. Sticking holds are checked after accepted
projections; work quadrature uses the correct side of each saved transition.

Immediately after release, slip can be smaller than floating-point directional
noise. Within the capture-speed numerical window, departure uses the consistent
Coulomb acceleration direction and projects accepted velocity onto its outgoing
ray in the physical mass metric. The ray includes zero, so this correction cannot
increase kinetic energy. Its work joins the separate projection ledger. This
prevents transverse direction chatter without introducing viscous creep; the
startup window is subject to the same independent tolerance refinements.

All force solves retain the `1e-10 N` certificate. Trial equations can extend
outside a mode's physical region while the integrator locates its boundary;
accepted states must pass the cone, passivity and zero-slip checks.

## Reproducible qualification

The expanded report covers static hold, slip arrest, reversal/re-sticking,
frictionless and frictional three-link arm contact, exact-sticking arm contact,
and two simultaneous arm contacts. Arm fixtures are free floating and include
contact onset, joint reaction transfer, off-center torque and separation.

Each report records all four maximum-step refinements (`2^-10` through `2^-13`),
independent work quadrature, exact-model capture refinements, integration-tolerance
and shifted-origin checks, full trajectories, immutable source hashes and
per-gate results. Both the half-second and two-second matrices are mandatory on
Linux and macOS. Budgets remain 1e-5 scaled energy-work error, 1e-7 scaled linear
and angular momentum error, and 1e-3 scaled trajectory/impulse agreement.
Projection energy is reported separately and limited to 1e-10 J magnitude.

Compare matching clean CI artifact directories with:

```sh
julia examples/compare_expanded_contact_platforms.jl linux/expanded macos/expanded comparison.toml
julia examples/compare_expanded_contact_platforms.jl linux/expanded-long macos/expanded-long comparison-long.toml
```

The comparison checks source identity, trajectories, impulse, work and mode
transitions. A passing local case alone does not complete
[Issue #4](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/issues/4);
all [expanded completion gates](design/contact_scope_expansion.md) must pass.
