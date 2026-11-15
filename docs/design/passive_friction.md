# Passive regularized friction for the first coupled configuration

The user-approved scope for Issue #4 is approximate sticking with bounded creep.
Exact zero-slip sticking remains a separate feature. The constitutive model below
replaces the unqualified native soft-friction solve; it is not force clipping or
a looser acceptance test. It applies to the existing one-box/one-sphere,
two-free-body configuration, with core retaining all integration.

## Native inputs and the coupled law

MuJoCo 3.11 supplies collision geometry, the dense contact Jacobian `J`, contact
velocity `v = J*qvel`, normal regularization `R_n`, normal reference acceleration
`a_ref,n`, and smooth generalized acceleration `a_0`. The provider calls the
position, velocity, actuation and smooth-acceleration stages, without calling the
native constraint solver or any time integrator. The actuator-free generated
model has no control callbacks or other native force generators.

Let `W = J*M^-1*J'`, using the same physical masses/full inertias as core, and
`b_n = J_n*a_0 - a_ref,n`. For coefficient `mu >= 0` and slip transition speed
`v_s > 0`, define

```text
d(v_t) = -mu * v_t / max(v_s, norm(v_t))
f_t = f_n * d(v_t)
0 <= f_n  perpendicular to  (W_nn + R_n)*f_n + W_nt*f_t + b_n >= 0
D = W_nn + R_n + W_nt*d(v_t)
f_n = max(0, -b_n/D)
```

Both components are solved together: tangential loading changes the normal
response through `W_nt`. A solve is supported only when
`D > sqrt(eps(Float64))*(W_nn + R_n + abs(W_nt*d))`; otherwise it is explicitly
refused. This bounds the numerical conditioning and excludes nonpositive
normal/friction response. It is not a general solution to frictional contact
indeterminacy for arbitrary mechanisms or multiple contacts.

For `mu=0` this reduces to the original scalar soft-normal equation, independently
checked by the head-on reference. The returned contact forces update the native
diagnostic force/acceleration fields consistently, then are projected to both
COMs. Only the returned wrenches enter the core dynamics, once per query.

## Dissipation and approximate sticking

For every admissible state, including slip reversal,

```text
norm(f_t) <= mu*f_n
f_t · v_t = -mu*f_n*norm(v_t)^2 / max(v_s, norm(v_t)) <= 0
```

Above `v_s`, friction has Coulomb magnitude and opposes slip. Below `v_s`, it is
viscous with coefficient `mu*f_n/v_s`. A subcritical tangential load therefore
produces finite creep, not exact sticking. The law is continuous at zero and at
the transition; its derivative can change at the transition. No hidden native
history is needed, so rejected stages and A–B–A queries remain state-pure.

For fixed normal preload and a scalar tangential load fraction `alpha` of the
Coulomb limit, the subcritical steady speed is `alpha*v_s`. Starting from rest,
the reduced unit-effective-mass benchmark has time constant
`tau = v_s/(mu*f_n)`, velocity `alpha*v_s*(1-exp(-t/tau))`, and displacement
`alpha*v_s*(t-tau*(1-exp(-t/tau)))`. Supercritical loads reach `v_s` and then
accelerate under saturated friction; the benchmark checks that transition too.

The default `v_s=0.001 m/s` sets a 1 mm/s creep-speed bound. Over two seconds
spent entirely in the creep regime, accumulated creep is bounded by 2 mm.
The trajectory report measures the actual creep interval and displacement;
this bound is not a claim of zero drift during long-term holding. The reduced
benchmark sweeps positive/negative subcritical and supercritical loads and
`v_s=0.0005, 0.001, 0.002 m/s`. Smaller values reduce creep but increase stiffness.
Regularized friction with an explicit stiction velocity is also described in
[Drake's continuous contact model](https://drake.mit.edu/doxygen_cxx/group__mbp__continuous.html).
The values here define a bounded computational model, not a material calibration.

## Certificate and qualification

The provider checks the original normal complementarity equation with a
force-valued projected residual, the tangential constitutive equation, and the
friction-cone bound. The maximum residual must remain below `1e-10 N` (or a
requested tighter limit). Native warnings, invalid states, unexpected constraints,
nonfinite outputs and ill-conditioned normal response are refusals. The obsolete
native Newton iteration/tolerance keywords are removed from the experimental
constructor because that solver no longer supplies the force.

Tests compare the coupled solution with a simultaneous BigFloat equation solve,
exercise both slip directions, zero slip, offsets and smooth loads, and reject
perturbed forces and inadmissible response. A separate RK4 reduced-coordinate
benchmark checks motion against analytic creep/sliding solutions, including
external work, dissipation, integration refinement, speed and displacement.
The native-input stages are independently compared with full `mj_forward`.

The original positive-power and positive-work fixtures are retained with passing
assertions required. Trajectory qualification still requires the original
momentum, angular momentum, energy-work, force/moment balance, cone, step and
tolerance-refinement budgets on Linux and macOS. Friction power is now checked
at all sampled speeds, not only the sliding classifier. The work quadrature must
resolve the faster creep transient independently of the core trajectory.

The old native counterexample (about +0.013129 W in sliding and +2.32e-6 J of
short-interval low-slip work) remains historical evidence explaining the model
change. It must not be relabeled as a passing native-friction qualification.
