# Experimental bus–bumper engine coupling

`BusBumperContact` implements `SpaceAGORA.ContactLoadServices v0.1.0` for one
rigid bus and one compliant attachment. Core owns both bodies and integration.
MuJoCo supplies collision geometry, contact kinematics and smooth acceleration;
the companion solves the coupled soft-normal/passive-regularized-friction law
and returns balanced contact-only COM wrenches. No native time step is called.

The default released-core pin remains unchanged and does not provide this
interface. Existing proximity scenes still work there; `BusBumperContact`
requires the merged contact-service revision used by experimental CI:

```sh
SPACEAGORA_CORE_REV=7c83bbf86499833addbba8a76ed7b6dfc743295e \
  julia --startup-file=no scripts/setup_env.jl /tmp/contact-env
julia --startup-file=no --project=/tmp/contact-env test/runtests.jl
julia --startup-file=no --project=/tmp/contact-env examples/compliant_contact.jl /tmp/contact-engine.toml
```

## Supported configuration

Use a box centered on the bus COM and a sphere centered on its sole attachment
COM. Mass and full inertia come from core; the same physical bodies supply the
native effective mass. The generated scene has no gravity, actuators, hidden
springs or native integration. Core refuses unsupported devices and combinations.
Start with serial Tsit5, one isotropic fixed-rest joint, and maximum core steps
of `2^-12 s`. Keep the native reference timestep fixed during core refinement.

The companion resets scratch on every trial state, refuses warnings, nonfinite
values, unexpected constraints and an invalid force certificate, and closes its
per-run runtime on success or failure. Independent force and common-origin
moment balance remain required. It does not support arbitrary geometry or
multiple contacts.

## Passive friction and finite creep

`BusBumperContact(friction=0.4, slip_speed=0.001)` uses Coulomb-magnitude friction
above 1 mm/s and a passive viscous approximation below that speed. Friction power
is nonpositive by construction. Normal and tangential loads are solved together,
including their effective-mass coupling. A nonpositive or ill-conditioned normal
response is refused.

**Sticking is approximate.** The default permits creep up to 1 mm/s, with a
2 mm upper bound over two seconds entirely in the creep regime. Actual creep
speed, duration and accumulated displacement must be measured for each fixture.
A smaller `slip_speed` reduces creep and increases stiffness; it is a physical
model parameter, not a solver tolerance. True zero-slip holding is outside this
first configuration.

The [law, dissipation proof, residual certificate and reference benchmarks](design/passive_friction.md)
state the equations and qualification scope. The constructor's former native
`iterations` and `tolerance` parameters are replaced by `slip_speed`; the native
friction solver is no longer called. The force certificate retains its maximum
`1e-10 N` budget, with tighter limits available through `residual_limit`.

The [expanded validation report](design/contact_validation_results.md) records
trajectory, creep, rejection/replay and cross-platform qualification. Issue #4
remains open for broader configurations and exact sticking.

## Validation and historical failure

Tests retain head-on scalar-reference agreement, off-center recoil, full-inertia
spin, force projection, momentum/angular momentum, frame changes, trial-state
purity and cleanup. New tests compare the coupled force with a BigFloat reference,
sweep slip direction and load, and measure creep/sliding motion against analytic
solutions. The two-second head-on example writes refinement and work ledgers.
The broader Issue #4 validation report requires independent quadrature refinement
and both short and two-second trajectories on Linux/macOS before qualification.

The former native soft-friction law produced approximately +0.013129 W along
slip and +2.32e-6 J of low-slip work in the original counterexamples. Those
failures motivated the explicit model change; they are not waived or reclassified.
The same physical fixture inputs remain regression cases for the replacement law.

This continues [Issue #4](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/issues/4).
Exact sticking, external targets, gravity/actuation, multiple attachments and
articulated mechanisms require separate qualification.
