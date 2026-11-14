# Two-way compliant multibody–MuJoCo contact coupling

**Status: proposed architecture and validation plan, 2026-11-13.** This is the
first design deliverable for [issue #4](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/issues/4),
not an implemented API or a claim of numerical validation. The existing
external-owner refusals remain in force. Completing the first configuration
below does not establish support for arbitrary robot arms or close the whole issue.

## Decision and first configuration

Keep SpaceAGORA as the only integrator of every physical body. Use MuJoCo as an
algebraic contact-load evaluator: synchronize a complete trial state, solve for
contact forces without advancing native time, and return those forces and their
moments to the core equations at that same trial state.

The first supported configuration should be **one freely moving rigid spacecraft
bus, with one generic `CompliantAttachment` containing one body, contacting a
stop rigidly attached to the bus**. Both bodies have finite mass and inertia;
the base is not fixed. Use full translation and attitude, constant physical
properties, fixed joint rest state, isotropic spring/damping coefficients,
zero translational damping (see the prerequisite below), one explicit
sphere–box collision pair, and the core `:tsit5` solver. Begin in
an inertial, gravity-free environment without controls or other effectors.

This bumper fixture exercises contact, compliance, bus recoil, mount reactions,
and off-center moments while staying within the current state layout. Core
[attachment validation](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/ebdd70120fb7d77ebe5dcf9b7ee754de57a698e4/src/orchestration/simulation/engine/setup.jl#L2833)
requires every spacecraft in a run to have the same attachment-body count.
A separate bare target spacecraft therefore needs additional state-layout work;
it is not part of this first configuration.

Initially refuse articulated roots, the separate legacy robot-arm path, multiple
attachments, arbitrary meshes, unlisted collision pairs, changing mass/inertia,
scheduled joint rest changes, actuators, external propagation owners, checkpoint
resume, mission events, flat/split/multirate RHS routes, implicit/AD solvers,
and concurrent RHS evaluation within a run. Preserve the ordinary no-contact
configuration and its behavior. A fixed wall is a diagnostic fixture with an
external reaction, not a substitute for the finite-mass bus acceptance test.

## Baseline and boundaries

The design was traced against companion main
[`8bae941`](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/tree/8bae94137e1ddfe654013ee60f131fecdb881084)
and core main
[`ebdd701`](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/tree/ebdd70120fb7d77ebe5dcf9b7ee754de57a698e4).
The companion currently pins core `bec28fcc` in `SpaceAGORASource.toml`; this
proposal requires a new coordinated core/companion source pair. It does not
claim that the proposed service exists in that release pin.

| Existing component | Consequence for this work |
|---|---|
| [External scene ownership](../architecture.md#stage-2-scenes-inside-run_simulation) | Advances native bodies after accepted core steps. It cannot supply state-pure trial contact loads. Keep this mode separate. |
| [Attachment load assembly](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/ebdd70120fb7d77ebe5dcf9b7ee754de57a698e4/src/models/dynamics/multibody/compliant/compliant_attachment_dynamics.jl#L177) | Currently assembles loads and immediately assigns attachment derivatives. Split those phases to insert contact. |
| [Joint load kernel](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/ebdd70120fb7d77ebe5dcf9b7ee754de57a698e4/src/models/dynamics/multibody/compliant/compliant_multibody.jl#L831) | Clears body load buffers, then calculates joint loads and mount reactions. Contact must be added after this reset. |
| [Relative acceleration completion](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/ebdd70120fb7d77ebe5dcf9b7ee754de57a698e4/src/models/dynamics/multibody/compliant/compliant_attachment_dynamics.jl#L207) | Must subtract the final bus acceleration, including contact, exactly once. |
| [Native binding](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/blob/8bae94137e1ddfe654013ee60f131fecdb881084/src/binding.jl#L26) | Has `mj_forward`, but needs verified contact records, contact-force extraction, generalized constraint loads, and solver diagnostics. |
| [Core Jacobian construction](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/ebdd70120fb7d77ebe5dcf9b7ee754de57a698e4/src/orchestration/simulation/engine/execution.jl#L1172) | Future cross-spacecraft contact must declare cross-body dependencies before enabling implicit solvers. |

Use “compliant multibody dynamics” in documentation; existing `Cloth*` symbols
are implementation names, not a reason to replace the physical model.

### Prerequisite: joint damping and angular balance

The current [joint force law](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/ebdd70120fb7d77ebe5dcf9b7ee754de57a698e4/src/models/dynamics/multibody/compliant/compliant_multibody.jl#L478)
applies opposite translational damping forces at distinct anchor points, with
opposite rotational couples. For isotropic translational damping `c`, its net
internal moment is `-c * (delta_r × delta_v)`. It is not generally zero.
For example, `delta_r=(0.01,0,0)` m, `delta_v=(0,0.1,0)` m/s and
`c=0.2` N·s/m give a residual moment `(0,0,-0.0002)` N·m. This follows from
the inspected force law; it is not a measured coupled-simulation result.

Add a core-only algebraic regression before contact integration. The first
qualified configuration requires zero translational damping; retain nonzero
rotational damping and dissipative contact. Treat noncentral translational
damping as an explicit characterization/expected-refusal case until a physically
consistent joint law and its work ledger are derived and tested. Do not hide
this intrinsic residual in a contact tolerance or count it as an external torque.

## Ownership and effective mass

| Quantity or operation | Owner |
|---|---|
| Physical state, accepted time, integration and output | Core |
| Mass/inertia definition, joint compliance and mount reactions | Core |
| Smooth physical forces, including future qualified gravity/actuation | Core |
| Collision geometry, contact regularization and contact solve | Companion/MuJoCo |
| Contact wrench application to physical bodies | Core, once per RHS evaluation |
| Temporary native state and solver workspace | One companion runtime per run; no global mutable scene |

Generate one native free body at each physical body's center of mass (COM).
Copy its mass and full inertia, with explicit inertial data and no mass inferred
from collision geometry. Do not include bumper mass again in bus mass. These
native inertias represent the same physical bodies for the contact solve;
there is no second integrated mass ledger. A mocap or kinematic proxy would
give the wrong effective mass and is unsuitable for either participant.

The generic attachment's compliance is a force law between independent rigid
bodies, so free-body inertias are an appropriate starting representation. This
argument does not extend to rigid articulated chains: their constrained mass
matrix and contact Jacobian would need a separate design.

Before solving contact, supply **all smooth physical wrenches** from the core,
including both sides of the compliant joint. Contact depends on the free
acceleration as well as geometry and velocity. Native gravity, springs, dampers,
actuators, joint friction, armature, limits, and equality constraints must be
absent or disabled. Preserve rigid-body inertial/gyroscopic terms; do not also
send a gyroscopic pseudo-torque as an applied physical torque. Verify the
contact-free native acceleration against the core before enabling contact.
This is a feasibility gate, not an assumed equivalence. See MuJoCo's
[constraint dynamics](https://mujoco.readthedocs.io/en/3.11.0/computation/index.html#constraint-model).

Return only contact contributions, not native total acceleration or the smooth
forces just supplied. With only free joints and contacts, compare the aggregate
generalized constraint force against independently summed per-contact wrenches.
Any additional native constraint invalidates that assumption and must be refused.

## Proposed service and RHS sequence

Introduce an experimental, versioned, core-owned contact-load service, provisionally
`ContactLoadServices`. Names below describe the proposed contract; they are not
callable APIs. Follow the existing adapter pattern: core owns types and hook
declarations, companion checks the version before extending hooks, and core
does not import MuJoCo. Do not overload `ExternalPropagationServices` 2.0.0 or
its experimental lifecycle with another meaning of dynamics ownership.

The contract needs:

- Configuration/preflight: stable body IDs, explicit collision pairs, immutable
  mass/inertia and geometry, settings, and supported-mode checks.
- Prepare/close: per-run native model and scratch ownership, cleanup on success
  and exception, and incompatible-version refusal before allocation.
- Evaluate: complete same-time body poses/velocities and smooth COM wrenches in;
  contact-only COM wrenches and convergence diagnostics out. Inputs are read-only.
- Reset/replay: restore scratch to a canonical state; no accumulated impulses or
  time counters. Diagnostics must not change the force result.

For each core RHS call `(u, t)`:

1. Gather all participating body states from that trial state. Assemble joint
   and other smooth loads, including mount reactions, without assigning body
   derivatives yet.
2. Reset native scratch to a known baseline. Set every pose, velocity and native
   time from this snapshot; clear applied-force, control, activation and solver
   history buffers. Disable warm starts and sleeping. Set the smooth wrenches.
3. Run `mj_forward`, extract contact-only loads, and check finite outputs,
   warnings, capacity and solver convergence. Fail explicitly on an invalid
   solve; never reuse the previous valid force.
4. Add returned loads once to each participant. Compute bus and attachment
   derivatives. Complete attachment relative accelerations using the final
   bus acceleration, including the bus contact load and its joint reaction.
5. Return derivatives only. Accepted core states alone define the trajectory.
   Produce diagnostics from explicit output-state queries or accepted-step data,
   never from whichever trial evaluation happened last.

`mj_forward` evaluates dynamics without integration; `mj_contactForce` returns
a six-component wrench in the contact frame. The binding must establish its
body-side/sign mapping with fixtures, not infer it from an array name.
[MuJoCo 3.11 function reference](https://mujoco.readthedocs.io/en/3.11.0/APIreference/APIfunctions.html#mj-forward)

No `mj_step`, `scene_step!`, accepted-step catch-up, or force cache keyed only by
time belongs in this path. A rejected step or a different state at the same
time must leave the next evaluation unaffected. Start with a serial RHS and
isolated runtime instances; qualify parallel execution separately.

For bus–bumper contact, the bus receives its native contact reaction **and** its
existing compliant-joint reaction. These are different physical loads. Do not
also negate the bumper contact wrench and inject it at the mount: the native
pair already accounts for the contact counterpart, while spring transmission
is accounted for by the compliant joint.

## Frames, points, units and time

Let `R_i` map body coordinates to inertial coordinates. Core quaternions are
scalar-last; native quaternions are scalar-first, representing the same physical
orientation. Disable native free-body alignment when generating the model and
verify the compiled COM, body frame and inertia. For the generic attachment,
translation is relative to the bus in inertial axes, while attitude and body
angular velocity are absolute:

```text
r_i = r_bus + att_r_i             v_i = v_bus + att_v_i
q_i = att_q_i                     omega_I_i = R_i * att_omega_i
```

Use native world axes parallel to inertial axes, with an explicitly fixed or
constant-velocity origin `O(t) = O0 + V0*(t-t0)`. Form native attachment positions
as `(r_bus - O(t)) + att_r_i` and velocities as `(v_bus - V0) + att_v_i` to avoid
subtracting nearly equal orbital positions. Do not reuse the accelerating chief
or differential-gravity stepping of `ProximityScene` in this mode.

Return `F_I` and `tau_I` about each body's COM; core body torque is `R_i' * tau_I`.
For a force at contact point `p` and a pure contact couple `m_i`, use
`tau_I_i = (p-r_i) × F_I_i + m_i`. Opposite contact forces/couples do not imply
opposite torques about different COMs. Check angular balance about a common
origin. Use meters, seconds, kilograms, newtons and newton-meters. Impulse is an
integral of force, reported in N·s; it is not a second load to apply.

Evaluate contact at every RHS stage, with no held-force exchange interval.
Keep native `opt.timestep` fixed for a run as a contact-solver setting; do not
replace it with an adaptive solver's changing trial step. Initially cap the
core step at `h = 2^-12 s`. Refine the core cap independently of native settings
and physical contact softness. Explicitly record `solref`, `solimp`, friction,
cone, solver, tolerance, iteration cap and collision settings.

The native `refsafe` option can clamp positive `solref[0]` to twice the native
timestep; ensure this clamp is inactive throughout the intended refinement
study. Warm starts and automatic free-body frame alignment also need explicit
settings for reproducible state queries.
[MuJoCo 3.11 XML options](https://mujoco.readthedocs.io/en/3.11.0/XMLreference.html#option-flag)

## Reproducible fixture

These are proposed test inputs, not measured or tuned validation results.

| Parameter | Initial value |
|---|---|
| Bus | 20 kg; centered box, full dimensions `(1, 0.8, 0.6)` m; inertia `diag(5/3, 34/15, 41/15)` kg·m² |
| Bumper | 1 kg; sphere radius 0.1 m; inertia `0.004 I` kg·m² |
| Mount and child anchor | Bus mount `(0.5, 0, 0)` m; bumper anchor `(-0.11, 0, 0)` m; coincident at rest |
| Initial relative pose | Bumper COM `(0.61, 0, 0)` m, identity attitude; 0.01 m surface gap |
| Initial velocities | Relative velocity `(-0.25, 0, 0)` m/s; bus velocity `(0.25/21, 0, 0)` m/s; zero angular rates; total momentum zero |
| Joint | Translational stiffness `200 I` N/m, zero translational damping; rotational stiffness `2 I` N·m/rad, damping `0.02 I` N·m·s/rad; fixed rest orientation |
| Contact | Explicit bus–bumper pair; zero margin/gap; `condim=1` frictionless, then `condim=3` with sliding coefficient 0.4; elliptic cone |
| Regularization | Positive `solref=(0.01, 1)`; constant impedance `solimp=(0.9, 0.9, 0.001, 0.5, 2)`; validate compiled settings in the feasibility harness |
| Numerics | Float64; native timestep `2^-12` s; Newton solver, tolerance `1e-12`, iteration cap 100; disable history-dependent features; record convergence diagnostics |
| Duration/refinement | 2 s initially; core maximum steps `2^-10, 2^-11, 2^-12, 2^-13` s; native timestep unchanged; dimensionally scaled core tolerances tightened independently |

The reduced translational mass is `20/21` kg. Initial relative kinetic energy is
about 0.02976 J, versus 0.01 J of translational spring energy at first geometric
touch; this checks the proposed fixture's approach energy. The simulation must
verify that contact actually occurs.
Use a mount offset of 0.2 m in the bus y direction for the off-center variant,
and a tangential initial velocity for friction tests, recording their changed
initial momentum and angular momentum rather than assuming they remain zero.

## Validation matrix and acceptance targets

Targets below are prospective gates. Publish measured values and convergence
curves before claiming support; do not silently relax thresholds to fit results.

| Test | Evidence and pass condition |
|---|---|
| Native binding and inertias | On Linux x86_64 and macOS arm64, verify 3.11.0 offsets/record strides, contact-frame ordering, body IDs, compiled mass/COM/inertia, and use-after-close handling. |
| Free dynamics and effective mass | Match core/native contact-free accelerations for force, torque and nonzero spin. Independently calculate `J M^-1 J'`, including off-center rotational terms, and match the native representation to `1e-10` scaled error. |
| Load projection | One known contact force/couple: match `mj_contactForce` summation to generalized constraint loads and core acceleration injection to `1e-10` scaled error. Check pair force and common-origin moment balance at the same tolerance. |
| No contact | Collision-disabled new path matches ordinary core dynamics under the same tolerances. Configurations without the new service retain existing bit-identical regression behavior. |
| Joint-only prerequisite | Independently sum anchor-force moments and rotational couples. Verify closure with zero translational damping, reproduce the noncentral damping residual above, and refuse that unqualified configuration. |
| Onset and separation | Head-on and grazing fixtures enter and leave contact; no stale load after separation, no attractive normal contact force beyond numerical tolerance, finite loads/penetration. |
| Frictionless response | Compare a 1D fixture to an independent implementation of the pinned scalar soft-constraint equation; verify effective mass, normal impulse and departure speed. Do not substitute a hard, perfectly elastic impact model. |
| Frictional response | Sliding and stick/slip transitions; verify friction-cone bounds, opposing slip power where sliding, angular response, and convergent tangential impulse. |
| Moving base and mount | Head-on recoil and off-center bus rotation; contact and joint loads separately instrumented; total momentum/angular momentum close their ledgers without duplicated mount loads. |
| Frames | Translate, rotate and apply a constant Galilean boost to a fixture; transformed results agree within the trajectory target. Include a large inertial position offset to expose cancellation. |
| Trial-state purity | Evaluate A–B–A, reversed times, and different states at identical times; A gives identical loads on the same host. Force solver rejection and compare the accepted trajectory with a refined reference. Native time equals the queried time and never becomes an independently advanced clock. |
| Reset/isolation/failure | Repeated reset/replay agrees on the same host; interleaved separate runtimes do not interfere. Reject unsupported configurations, bad IDs/inertias, nonfinite inputs, native warnings/capacity exhaustion and unconverged solves with actionable errors. |
| Refinement | At common output times, finest two resolutions differ by less than `1e-3` in scaled pose/velocity and integrated contact impulse; show decreasing errors across the full sweep. Tighten core tolerances, then vary native timestep and solve tolerance separately. Report peak penetration, contact duration and peak load without assuming smooth-event convergence order. |

Use fixed nonzero scales: `M*=21 kg`, `L*=1 m`, `V*=0.25 m/s`,
`P*=M*V*`, `H*=M*L*V*`, `E*=M*V*^2`, `F*=M*V*^2/L*`, and `tau*=F*L*`.
Normalize each metric by its appropriate scale, including quaternion angular
error in radians; use `V*/L*` for angular velocity. Compare impulses using `P*`.
For algebraic mass-matrix tests, scale translational and rotational coordinates
consistently by `L*`. This avoids dividing by initially zero momentum or energy.

At the finest converged resolution, target maximum momentum and angular-momentum
ledger residuals below `1e-7` of their scales and energy-work residual below
`1e-5 E*`. Require convergence of independent work quadrature as well as the
trajectory. Native iteration count alone is not a convergence certificate:
the feasibility harness must choose and record a residual/optimality diagnostic
with a quantified contribution below the load-projection error budget.
Bitwise reproducibility is a same-host test; cross-platform comparisons use the
stated numerical tolerances.

## Momentum and energy accounting

Sum each physical body once. With inertial velocities and a fixed inertial
origin `O`, compute

```text
P = sum(m_i * v_i)
H = sum((r_i-O) × (m_i*v_i) + R_i*I_i*omega_body_i)
T = sum(0.5*m_i*dot(v_i,v_i) + 0.5*dot(omega_body_i,I_i*omega_body_i))
Delta(T + U_joint + U_gravity) = W_external + W_actuation + W_contact - D_joint
W_contact = integral sum(F_contact_i · v_i + tau_contact_i · omega_I_i) dt
```

Integrate momentum ledgers against external force and moment. Internal joint
and contact pairs cancel in the closed bus–bumper system. A fixed-wall test has
an external impulse; a moving prescribed wall can also do work. Record those
boundary terms explicitly instead of requiring closed-system conservation.

For the initial isotropic, fixed-rest joint and modest rotation, derive and
test `U_joint = 0.5*delta_r' K delta_r + 0.5*k_r*|rotation_log|^2` and the
corresponding nonnegative damping loss directly against the implemented joint
wrench. Do not assume this expression covers arbitrary anisotropic finite-angle
laws. Keep initial angular fixtures below pi/4 and avoid the rotation-log branch.
Actuation and gravity work/potential are zero in the first fixture but remain
explicit ledger terms; enabling either needs a dedicated validation fixture.

The contact-work ledger checks numerical bookkeeping, not physical validity by
itself. Compute work with independent output-state queries and refined
quadrature, and separately test dissipative behavior. Normal contact may return
stored energy while unloading; do not require instantaneous kinetic-plus-joint
energy to decrease during contact or invent an unverified MuJoCo contact
potential. Between separated endpoints, an unforced dissipative fixture must
not gain mechanical energy beyond the converged error budget. Measure effective
restitution from approach/departure speeds and sweep contact parameters; do not
equate `solref` damping with a specified restitution coefficient.

An all-MuJoCo comparison using matching smooth forces is useful for adapter
consistency, but shares the contact implementation and is not an independent
physical reference. Keep analytic inertia/impulse checks, the scalar reference,
core-only baselines, and conservation/work ledgers as separate evidence.

## Delivery gates

1. **Joint-law prerequisite and standalone feasibility.** Characterize the
   core-only damping residual above and enforce the bounded initial scope.
   In the companion, add verified binding access and a
   standalone state-to-contact-load harness. Establish free-acceleration parity,
   effective mass, contact signs/moments, scalar response, solve convergence and
   A–B–A purity. Stop and revisit the design if any cannot be established.
2. **Core contract and load assembly.** Introduce the experimental service and
   split smooth-load assembly from derivative assignment. Use a deterministic
   fake provider to test force injection, mount accounting, reset/error handling,
   refusals and no-contact identity without requiring MuJoCo in core CI.
3. **First coupled configuration.** Implement the companion adapter, executable
   bus–bumper example, full matrix above and a measured numerical report. Run
   mandatory native tests on both supported platforms with
   `SPACEAGORA_MUJOCO_REQUIRED=1`. Pin the compatible core release and companion
   revision together after both changes pass their own checks.
4. **Scope expansion, separately qualified.** Plan heterogeneous spacecraft
   state blocks and cross-body RHS assembly; then external-target contact,
   orbital gravity, actuation work, implicit solvers, articulated effective mass,
   and the robot-arm path. Each expansion needs its own fixture and refusal
   changes. Review the remaining issue #4 checklist before proposing closure.

Keep the measured report beside this design, with immutable source revisions,
native artifact/platform/Julia versions, full solver/contact settings, commands,
machine-readable trajectories and ledgers, convergence plots, measured errors,
runtime cost and explicit limitations. CI should run small deterministic
regressions; retain the longer refinement sweep as a reproducible report command.
This planning change itself adds no coupling code and supplies no measured
simulation results.
