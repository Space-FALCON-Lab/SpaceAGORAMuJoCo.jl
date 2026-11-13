# Initial contact feasibility results

This is the first implementation gate of the
[issue #4 design](compliant_contact_coupling.md), not an end-to-end coupling
validation. The implementation is a bounded research harness under `test/support/`;
it is not a public contact service and does not advance either engine's trajectory.

## Reproduce

From this repository, with Julia 1.12 and a C compiler:

```text
julia --startup-file=no test/contact_feasibility_standalone.jl report.toml
```

The driver loads the pinned native artifact, checks its headers against every
bound offset and struct size, runs all native feasibility assertions, and writes
measurements only if the tests succeed. It requires the native/ABI checks rather
than silently skipping them. The optional report records source revision,
working-tree cleanliness, native artifact tree, Julia version, platform and UTC
time. Both supported-platform CI jobs run the normal suite and upload this report.

The normal package suite additionally runs `test/contact_core_parity_tests.jl`,
comparing actual core and native free-body accelerations under the same COM
wrenches. This test uses the public compliant-model API and includes rotated
anisotropic inertia, nonzero spin and applied torque.

## Measurements

The [recorded macOS arm64 run](contact_feasibility_macos.toml) used Julia 1.12.1
and MuJoCo 3.11.0. Its provenance identifies the exact clean implementation
revision before this report was added. All 175 standalone assertions passed.
The direct core-parity test passed its five assertions locally against the core
damping branch; the normal CI suite checks the companion's pinned core release.
The existing 54 binding assertions also passed locally.

| Quantity | Measured result |
|---|---|
| Centered effective mass | `0.9523809523809523 kg` (`20/21`) |
| Independent scalar normal force | `57.14285714285713 N` |
| Native normal force | `57.14285714285743 N` |
| Maximum scalar force error, nine cases | `2.99e-13 N` |
| Off-center normal inverse mass | `1.0646341463414635 kg^-1` |
| Frictionless projected optimality residual | `3.56e-14 N` |
| Frictional projected optimality residual | `6.08e-15 N` |
| Contact force projection residual | zero in the recorded cases |
| Maximum torque projection residual | `3.56e-15 N m` |
| Common-origin pair moment residual | zero in the recorded cases |

The sweep varies bus mass over 2, 20 and 200 kg, and native timestep over
`2^-10`, `2^-12` and `2^-14` seconds. Geometry, velocity and physical contact
settings are fixed: constant impedance 0.9, positive `solref=(0.01,1)`, zero
margin/gap, Newton solver tolerance `1e-12` and maximum 100 iterations. The
`refsafe` clamp is inactive. These are state-query tests, not a trajectory
step-convergence study.

The frictionless scalar reference computes the unilateral solution directly
from physical masses, gap, relative velocity, smooth force, impedance and
reference parameters. Off-center directional inverse mass is verified both
geometrically and through native free-acceleration responses to independently
constructed loads. The elliptic-friction test checks an independently evaluated
projected optimality residual with a `1e-10 N` rejection threshold; it does not
infer convergence from iteration count alone. A zero-iteration solve is rejected.

## Findings that affect the adapter

- The sphere–box collision returns sphere first even when the MJCF pair names
  the box first. Use the returned geometry IDs and frame for each contact's
  sign; do not rely on input order.
- With `diagexact` enabled, the normal regularization uses the exact directional
  inverse mass. For elliptic contacts, MuJoCo subsequently adjusts the
  tangential regularization entries; `efc_diagA` is therefore not a readback of
  every diagonal of `J M^-1 J'`. The tests verify that matrix independently.
  See the pinned [constraint implementation](https://github.com/google-deepmind/mujoco/blob/3.11.0/src/engine/engine_core_constraint.c).
- Supplying smooth physical loads changes the solved contact force. Returning
  only the constraint/contact contribution avoids applying those smooth loads
  twice. Gyroscopic terms agree between the core and native free-body equations.
- Resetting scratch and supplying the complete state makes A–B–A, reversed-time,
  same-time/different-state and interleaved-runtime queries repeat exactly on
  this host. Native time, poses and velocities do not advance. Rotated, translated
  and boosted fixtures produce the corresponding transformed loads.

The force reference follows the pinned
[soft-constraint formulation](https://mujoco.readthedocs.io/en/3.11.0/computation/index.html#parameters)
and [reference-parameter mapping](https://mujoco.readthedocs.io/en/3.11.0/modeling.html#solver-parameters).
It validates the adapter's implementation of that model, not its fidelity to
every physical contact material.

## Damping prerequisite and remaining gates

[Core PR #604](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/pull/604) corrects
the noncentral translational-damping moment identified in the design. It uses
velocity fields at a common midpoint and transfers the force moments to both
anchors. Its core-only regressions cover generic bodies, moving mounts and the
separate robot-arm path. In a 1 s damped fixture the maximum linear/angular
momentum errors are `6.09e-16 kg m/s` and `5.18e-16 kg m²/s`, with energy-work
residual `6.13e-11 J`. The source change and evidence are in that PR's review
record. The companion's core release pin has not changed.

Still required: the core contact-service contract, split load assembly,
end-to-end bus–bumper integration, adaptive rejection through the actual core
solver, trajectory convergence, and the full momentum/energy/contact-work
report. No gravity, actuated compliance, articulated-root or robot-arm contact
support follows from this feasibility result. Existing external-owner refusal
checks stay in place, and issue #4 remains open.
