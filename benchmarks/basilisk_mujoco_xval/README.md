# SpaceAGORA ProximityScene vs Basilisk 2.12 MJScene

An accuracy cross-validation (not a timing study) of `SpaceAGORAMuJoCo.jl`'s `ProximityScene` against the
MuJoCo-based `MJScene` of Basilisk 2.12. Both tools run MuJoCo 3.11.0 on the same MJCF, the same point-mass Earth
field (mu = 3.98600436233e14 m^3/s^2, SpaceAGORA's Earth, also set on the Basilisk body) and the same initial state:
400 km circular orbit, 51.6 deg, (r, 0, 0) with v = sqrt(mu/r)(0, cos i, sin i).

Acceptance criterion: for each case and quantity, a dt-halving self-convergence study on each side, and
D <= k max(e_S, e_B) with k = 10, where D is the cross-tool difference between the finest runs and e_S, e_B are the
differences between each tool's two finest runs. Every statistic is the maximum over the 1 s samples of the vector
2-norm. No floor or extra tolerance is applied.

## Files

| file | role |
|---|---|
| `cases/case{A,B,C}.toml` | case definitions (orbit, initial states, joint state, piecewise-constant actuator torque schedule, dt ladders) |
| `cases/*.xml` | MJCF models derived from Basilisk examples (see `NOTICE.md`) |
| `sagora_driver.jl` | SpaceAGORA side, one run per dt in `sagora_dt_s` |
| `bsk_driver.py`, `run_bsk.sh` | Basilisk side, one run per step in `bsk_dt_s` |
| `compare.py` | reads both ladders, writes `results/summary_<case>_T<s>.{csv,md}` |
| `run_case.sh` | the lab's wrapper: Basilisk on a remote lab machine, SpaceAGORA locally, then compare |

## Matching

- Frames: Basilisk uses absolute ECI coordinates; the SpaceAGORA scene uses a chief frame. Only relative quantities
  are compared: body-minus-hub COM position and velocity, hub attitude quaternion (MuJoCo order, sign-aligned) and
  body-frame rate, joint angle and rate. World axes are inertial on both sides.
- Gravity: Basilisk `NBodyGravity` (point mass, `m g(r_com)` at every body COM); SpaceAGORA
  `InverseSquaredGravityModel` through the Encke chief scheme. Same physics, different formulation.
- Actuation: the same piecewise-constant torque schedule through MJCF motors (`scene_ctrl` / `SingleActuatorMsg`),
  value held over each 1 s sample interval. Switch times are multiples of 1 s.
- Integrators (differ, by design): Basilisk integrates the MuJoCo accelerations with its own fixed-step RK4 (task step
  = integration step; `extraEoMCall` on). SpaceAGORA uses MuJoCo `implicitfast` at fixed dt with the gravity wrench held
  over the step (`mj_step1`/`mj_step2`), which is first order in dt. Hence the ladders: SpaceAGORA dt halved from
  0.04 s to 0.00125 s (Cases A, B; six levels) or 0.004 s to 0.0005 s (C; four levels), Basilisk step halved from 0.2 s (A, B) or 0.004 s (C), four levels each.
- Initial state: hub position/velocity are the hub COM (equal to the body origin in all three models); initial joint
  angles and rates are written into the SpaceAGORA scene's `qpos`/`qvel` after construction and set with
  `setPosition`/`setVelocity` on the Basilisk joints; the hub rate is the body-frame rate at identity attitude.
- Basilisk attitude output: `MJSite` writes the MRP of `site_xmat` (body-to-world) directly, so the MuJoCo quaternion is the
  plain MRP-to-Euler-parameter map (no conjugate).

## Integrator finding: `integrator=:rk4` is not RK4

The runner first accepted `integrator=:rk4`, but it advances with `mj_step1`/`mj_step2`, and MuJoCo's `mj_step2` does not
support RK4 (it falls back to Euler). The scene constructor now refuses `:rk4`. Before that fix, a Case A run with `:rk4` was bit-identical to one with `:euler` (checked with
`cmp` on the CSV outputs, dt = 0.02 s, 30 s). So no higher-order SpaceAGORA integrator is available without changing the
package. `implicitfast` gave the same Case B summary as `:euler` to the digits shown (these models have no damping or armature, so the implicit velocity derivative has nothing to act on), so
neither is an independent check. Instead `compare.py` adds second-order Richardson extrapolation over the three finest
levels (`D_extrap2_S`) and the observed order of D (`order D`); neither affects pass/fail.

## Cases

- A: hub plus one hinged panel (`panel_10` of `sat_w_branching_panels.xml`), contact disabled as published, motor
  torque +-0.25 N m pattern (period 80 s), joint from 0.8 rad (range 0..pi/2 never reached in 300 s), hub rate
  (0.002, -0.003, 0.001) rad/s. Run for 30 s and 300 s.
- B: `sat_w_thruster_arms.xml` verbatim (hub + two 4-joint arms), initial hub spin (0.01, -0.02, 0.015) rad/s, initial
  rates on all 8 joints, small torques on two joints. 30 s. Geoms cannot collide as published.
- C: `sats_dock.xml` with the `contact="disable"` flag removed: two 1 kg, 1 m cubes, off-center (0.3 m) approach at
  0.1 m/s with tangential rate and spin, 12 s, no weld latch. The file publishes no contact parameters, so MuJoCo
  defaults apply (solref 0.02 1, solimp 0.9 0.95 0.001, friction 1 0.005 0.0001). The latch (mid-run weld toggle) is not
  covered: it is Stage 3 of the SpaceAGORA scene.

## Running

From this companion repository, create the independent Julia environment with
`scripts/setup_env.jl` as described in the root README. A local comparison needs
a separate MuJoCo-enabled Basilisk 2.12 build and its Python environment. For
example, from this directory (replace paths and choose a case/duration):

```sh
BSK_DIST=/path/to/basilisk/dist3 BSK_PY=/path/to/basilisk/python \
  sh run_bsk.sh cases/caseA.toml results/local-bsk/caseA_T30 30
julia --project=/path/to/mjxval_env sagora_driver.jl \
  cases/caseA.toml results/local-sagora/caseA_T30 30
python3 compare.py cases/caseA.toml results/local-sagora/caseA_T30 \
  results/local-bsk/caseA_T30 results/summary_caseA_T30
```

These are scientific workloads: run them on an otherwise idle machine with
suitable resource limits. None of this is needed for installation or for the
synthetic CI checks.

Basilisk needs a build with MuJoCo enabled; a standard build has
`BUILD_MUJOCO=OFF`. The comparisons used a separate copy of the Basilisk 2.12
source (commit 611665f, clean tree) with its own Conan cache. The build needs
network access: it downloads MuJoCo 3.11.0, cspice and cfitsio, and takes SWIG
from Basilisk's virtual environment, whose `bin` must be on `PATH`. From that
source copy:

    PATH=/path/to/basilisk/.venv/bin:$PATH CONAN_HOME=/path/to/conan_home \
      python conanfile.py --mujoco True --vizInterface False --opNav False --buildTesting False

The commands above then run one case end to end. `run_case.sh <case> <duration_s>
<julia-env>` is the lab's wrapper for the same three steps: it runs the Basilisk
side on a remote lab machine over SSH, stops if that run fails or reports a
warning or instability, copies the results back, then runs the SpaceAGORA side
locally under a memory limit and the comparison. It depends on the lab's host
names and directories, so outside the lab run the three steps directly. The
SpaceAGORA side writes `results/<hostname>/<case>_T<s>/`, and the comparison
writes `results/summary_<case>_T<s>.{csv,md}`; `results/` is ignored by git.
Wall times from these runs are not performance results.

The comparator requires finite, matching sample times within and across ladders
(absolute tolerance 1e-9 s). From the companion repository root, the guard tests
run without any native or remote workload:

    python3 -m unittest discover -s benchmarks/basilisk_mujoco_xval -p test_harness.py

## Reading the results

`results/summary_<case>_T<s>.md` has, per quantity: D, e_S, e_B, bound, D/bound, observed order of each self-convergence
study, observed order of D versus SpaceAGORA dt, `D_extrap_S` / `D_extrap2_S`, and pass/fail. Because SpaceAGORA is first
order, D is about e_S by construction and the k = 10 test is weak for Cases A and B; `order D` and the extrapolated D
are the sharper evidence. Case C carries an explicit "inconclusive" verdict (`verdict` key in `cases/caseC.toml`).
