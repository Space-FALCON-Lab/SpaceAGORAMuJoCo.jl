# Expanded Issue #4 completion gates

The requested completion scope now includes the validated bus–bumper mode,
true Coulomb sticking, and the existing compliant robot-arm route with additional
collision primitives. This document supersedes the earlier proposal to close
the issue at bounded-creep qualification. None of the gates below is satisfied
merely by preserving the already passing regularized-friction tests.

## Dynamics and geometry

Core remains the sole owner of the bus and every compliant link's motion.
Extend the exact-version contact contract to an ordered body collection: bus
first, then attachment or arm state columns. Each body retains its own physical
mass and full COM inertia. Assemble every smooth joint load and mount reaction
before querying contact; add each returned COM wrench once before assigning
derivatives. Do not represent the existing compliant arm as a rigid articulated
chain with a different effective mass.

Qualify multiple compliant links and explicit collision pairs with sphere, box
and capsule primitives, including offset and rotated geometry. All participants
have finite physical mass. Unqualified meshes, external owners and articulated
backbones remain refused; 'additional geometry' does not imply that arbitrary
collision algorithms or topologies are physically validated.

## Exact sticking

The regularized model remains available with its explicit finite-creep meaning.
The exact-sticking model must instead use set-valued Coulomb friction: zero slip
can support a tangential load anywhere inside the friction disk; sliding lies
on the disk boundary and opposes slip. Reducing the old transition speed is not
an implementation of this requirement.

Contact-mode changes and any constraint projections must be owned by accepted
integrator transitions. Rejected stages, replay queries and same-time different
states cannot commit a mode, impulse or anchor. Solve the coupled normal and
tangential equations with an independently evaluated certificate. Include the
time-varying contact Jacobian in sticking acceleration constraints. Numerical
capture/projection tolerances must be exposed and independently refined; they
must not introduce a physical creep parameter.

## Required evidence before completion

1. Multi-link smooth-load parity with the existing arm implementation; balanced
   bus/mount reactions; identity with zero contact; replay and failure cleanup.
2. Native mass/Jacobian and COM-wrench projection for each primitive/pair,
   including simultaneous contacts and rotated full inertias.
3. Static holds below the friction limit with slip and displacement bounded by
   numerical tolerances, breakaway above the limit, slip arrest, reversal and
   re-sticking. Independently vary solver steps and capture/projection tolerances.
4. Floating-base arm onset/separation, joint reaction transfer, off-center torque,
   frictionless and frictional trajectories, and simultaneous-contact cases.
5. Closed linear/angular momentum and energy-work ledgers. Separate normal work,
   friction, joint damping, any actuation/external work and numerical projection
   impulses. Report projection energy instead of hiding it as friction loss.
6. Preserve existing force, moment, energy-work and convergence budgets. Publish
   new reproducible examples, complete short/long trajectories and immutable
   source provenance; require qualification on Linux and macOS and compare them.

The original merged baseline remains core `7c83bbf8` and companion `eb91139a`.
The expanded implementation and its qualification are in progress. Issue #4
must remain open until these gates pass.
