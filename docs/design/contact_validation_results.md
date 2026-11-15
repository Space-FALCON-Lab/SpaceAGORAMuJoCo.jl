# Passive friction qualification for the coupled bus–bumper fixture

The coupled provider uses passive regularized Coulomb friction, with approximate
sticking and measured creep. The former positive sliding-power and positive
low-slip-work counterexamples are required passing regressions. No physical
acceptance budget was relaxed. Exact zero-slip sticking is outside this first
configuration; Issue #4 remains open for broader support.

The [constitutive law and certificate](passive_friction.md) explain the coupled
normal/tangential solve, dissipation proof, conditioning restriction and analytic
reference. The default creep transition speed is 0.001 m/s: motion entirely in
the creep regime accumulates at most 2 mm over two seconds. This is a finite-drift
model, not an exact holding constraint or a calibrated material model.

## Reproduction and required gates

Install the exact core pairing in the [engine guide](../compliant_contact.md), then:

```sh
CONTACT_VALIDATION_TEST_REPORT=/tmp/contact-validation/tests.toml \
  julia --startup-file=no --project=/tmp/contact-env test/runtests.jl
julia --startup-file=no --project=/tmp/contact-env \
  examples/validate_contact.jl /tmp/contact-validation --require-qualified
CONTACT_VALIDATION_DURATION_S=2.0 julia --startup-file=no --project=/tmp/contact-env \
  examples/validate_contact.jl /tmp/contact-validation/long --require-qualified
python3 examples/plot_contact_validation.py /tmp/contact-validation
```

Both Linux and macOS CI require the short and full two-second matrices to pass.
The `contact-validation-<platform>` artifacts contain source revisions, clean-tree
flags, SHA-256 hashes, physical settings, all gates, refinement records, analytic
creep benchmarks and complete trajectory CSVs. `--require-qualified` exits nonzero
for a failed physical gate. There are no expected-failure friction assertions.

The five cases are head-on, off-center sliding, nonzero joint damping, low slip
and grazing near the box edge. Each runs at four core maximum steps from `2^-10`
through `2^-13` s. Native timestep stays `2^-12` s; native collision and smooth
acceleration stages do not integrate time or solve friction. Core owns both
physical bodies' integration. Independent tolerance/native-step studies, forced
rejection and bit-identical replay, rotated frames and tighter force certificates
are included in the test suite.

| Quantity | Unchanged acceptance budget |
|---|---:|
| Linear momentum residual | 5.25e-7 kg m/s |
| Angular momentum residual | 5.25e-7 kg m²/s |
| Energy minus contact/damping work | 1.3125e-5 J |
| Independent work-quadrature difference | 1.3125e-6 J |
| Scaled trajectory and impulse refinement | 1e-3 |
| Contact-force certificate | 1e-10 N |

Friction power and net friction work must be nonpositive within 1e-10 W/J.
The friction cone, creep speed and accumulated creep displacement are checked
for every case. The independent creep quadrature difference must be below 1e-6 m.
Force/moment balance and world-wrench versus constraint-power agreement remain
separate test requirements.

Work uses adaptive four-point Gauss quadrature with geometric contact-event
splitting. Adaptation resolves the rapid low-slip transient; it changes diagnostic
integration accuracy, not the core trajectory or energy-error acceptance budget.
Creep integrals count contact intervals below the configured transition speed.
They exclude ordinary sliding; total sliding displacement is not creep.

The reduced constant-preload reference independently checks exponential creep
and the transition to saturated sliding for both load directions. It sweeps
0.5, 1 and 2 mm/s transition speeds and subcritical/supercritical loads, comparing
the production force law integrated by RK4 against analytic velocity,
displacement and external-work balance. Full trajectories also sweep transition
speed. These and the normal-time-constant sweep change the physical model;
they are not numerical-convergence studies.

## Cross-platform comparison

Download Linux and macOS validation artifacts from the same CI run, then run:

```sh
julia --startup-file=no examples/compare_contact_platforms.jl \
  /tmp/contact-linux /tmp/contact-macos /tmp/contact-comparison.toml
julia --startup-file=no examples/compare_contact_platforms.jl \
  /tmp/contact-linux/long /tmp/contact-macos/long /tmp/contact-long-comparison.toml
```

The comparison rejects different source revisions, dirty trees, settings, hashes
or time grids. It checks both bodies' position, velocity, orientation, angular
rate and contact impulses against the unchanged 1e-3 scaled budget. Its numerical
agreement result and physical qualification result are separate and must both
pass before merging. CI artifacts are the authoritative per-revision evidence.

## Historical failures and scope

The [superseded native-friction report](native_friction_validation_results.md)
preserves the original +0.013129 W sliding-power counterexample, +2.32e-6 J
low-slip work, refused native solve and old cross-platform measurements. Those
snapshots describe the old native soft-friction law, not the current provider.
The new law solves the coupled normal response and passive friction together;
it does not clip a native result or reuse the old solver's certificate.

Qualification is bounded to one box bus and one spherical bumper, physical full
inertias, the existing compliant joint with small relative rotation, and admissible
single-contact response. Exact sticking, gravity, actuation, external targets,
multiple attachments, articulated chains and implicit solvers require separate
designs and validation. This work does not close Issue #4.
