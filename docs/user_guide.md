# Simulating orbital proximity scenes with MuJoCo

SpaceAGORAMuJoCo combines orbital motion with MuJoCo's rigid-body, joint and
contact dynamics. It is intended for scenes in which local geometry matters:
two nearby spacecraft, a free-flying vehicle with articulated links, or bodies
that may touch. SpaceAGORA supplies the gravitational field; MuJoCo evolves the
mechanical scene. You can step a scene directly or let it carry selected
spacecraft during a SpaceAGORA simulation.

The companion is optional. Installing or importing SpaceAGORA alone does not
load MuJoCo. The current native binding requires Julia 1.12 and Linux x86_64 and
uses the pinned MuJoCo 3.11.0 artifact.

## Choose the guide and source pair together

This version of the guide uses `ExternalPropagationServices` contract 2.0.0,
as the releases from v0.2.0 on do. The current checkout pins SpaceAGORA v0.7.0;
the existing companion v0.5.0 tag pins SpaceAGORA v0.6.0. The v0.1.0
release uses contract 1.0.0; if you install that release, use its
[tagged user guide](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/blob/v0.1.0/docs/user_guide.md)
and [release record](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/releases/tag/v0.1.0).
Do not combine its environment with the current coupled example below.

To follow the current guide, prepare an environment from the companion checkout
that contains it. For a new checkout of the default branch (add
`git checkout --detach v0.5.0` after `cd` to use the v0.5.0 release instead):

```sh
git clone https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl.git
cd SpaceAGORAMuJoCo.jl
unset SPACEAGORA_CORE_PATH SPACEAGORA_CORE_REV
julia --startup-file=no scripts/setup_env.jl /path/to/mujoco-env
JULIA_LOAD_PATH='@:@stdlib' julia --startup-file=no --project=/path/to/mujoco-env
```

Setup selects this checkout and the exact core revision in its `SpaceAGORASource.toml`.
Keep the checkout, environment and managed `sources/` directory available: the
manifest refers to those local paths. This is the current-checkout route, not a
claim that every default-branch revision is a qualified release. Record both
source revisions with a study. The [installation guide](../README.md#installation)
explains dependency setup and the distinction from published companion releases.

If you already have a SpaceAGORA source checkout, its
[Robotics package setup](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/main/packages/robotics/README.md)
can install the recorded companion under `packages/robotics/SpaceAGORAMuJoCo.jl`
into a separate environment. That route uses your core checkout; the standalone
installer uses `SpaceAGORASource.toml` on the development branch; the immutable
v0.5.0 tag records its historical SpaceAGORA v0.6.0 pin in `Project.toml`.

Start with the [complete standalone example](#a-complete-standalone-example),
then [carry it inside SpaceAGORA](#carrying-the-scene-inside-spaceagora).
Read [coupled output timing](#reading-coupled-output) and
[practical limits](#scope-and-practical-limits) before comparing results.

## A moving origin, with inertial axes

Orbital coordinates are large compared with the separations and dimensions of a
proximity scene. The runner therefore keeps MuJoCo's positions near a moving
reference point, called the *chief*. At reset, the chief is initialized at the
scene's mass-weighted center of mass with the corresponding velocity. It then
follows its own orbital trajectory. The local axes remain parallel to the
inertial axes: this is a translating frame, not a rotating orbital or LVLH frame.

Each body receives the difference between gravity at its own center of mass and
gravity at the chief. MuJoCo's built-in uniform gravity is disabled. This retains
the tidal acceleration across the scene while separating it from the large
common orbital acceleration. The public state accessors add the chief's motion
back, so the positions and velocities you provide and retrieve are **absolute
inertial states**. Do not subtract the chief before constructing a
`SceneBodyState`.

The conventions used by the public interface are:

| Quantity | Convention |
| --- | --- |
| `SceneBodyState.r`, `scene_body_state(...).r` | Body center-of-mass position, inertial axes, metres |
| `SceneBodyState.v`, `scene_body_state(...).v` | Body center-of-mass velocity, inertial axes, metres per second |
| Scene quaternion `q` | Unit quaternion `(w, x, y, z)` with `v_world = R(q) * v_body`, where `R` is the Hamilton rotation matrix and the world axes are parallel to the inertial axes |
| Angular velocity `ω` | Body axes, radians per second |
| Returned `ω_world` | The same angular velocity expressed in inertial-parallel world axes |
| Scene time and step | Seconds; `scene_time(scene) = t0 + n*dt` |
| MJCF mass and inertia | Kilograms and kilogram metres squared |
| External force supplied to a step | Newtons, inertial axes, applied at each body's center of mass |

An MJCF body origin may differ from its center of mass. Position and velocity
accessors refer to the center of mass; the quaternion describes the body's
axes. MJCF geometry is written in metres, and a box's `size` gives its
**half-lengths**. Set the MJCF compiler's angle convention explicitly when using
angle-valued model attributes.

SpaceAGORA spacecraft quaternions use scalar-last order, `(x, y, z, w)`, and
describe the same rotation as `v_body = rot(q) * v_inertial`, with
`rot(q) = R(q)'`. Use `sa_to_mujoco_quaternion` and
`mujoco_to_sa_quaternion` when crossing that boundary. The conversion reorders
components; it does not conjugate the quaternion or rotate the body angular
velocity. `body_state_from_initial_condition(name, ic)` performs this conversion
for a SpaceAGORA Cartesian initial condition.

## A complete standalone example

This example places two identical, non-contacting cubes 20 metres apart near a
7000-kilometre geocentric radius. The chaser receives the circular-orbit speed at
its initial radius; the target starts with the same velocity. These initial
conditions do not prescribe a formation-keeping trajectory. Their relative
motion is a result of the gravitational field.

Save the following as `two_cubes.jl` and run it in the installed companion
environment with `julia --project=/path/to/mujoco-env two_cubes.jl`.

```julia
using SpaceAGORA, SpaceAGORAMuJoCo
using StaticArrays, LinearAlgebra

xml = """
<mujoco model="two orbital cubes">
  <compiler angle="radian"/>
  <worldbody>
    <body name="chaser">
      <freejoint/>
      <inertial pos="0 0 0" mass="12" diaginertia="2 2 2"/>
      <geom type="box" size="0.5 0.5 0.5" mass="0"
            contype="0" conaffinity="0"/>
    </body>
    <body name="target">
      <freejoint/>
      <inertial pos="0 0 0" mass="12" diaginertia="2 2 2"/>
      <geom type="box" size="0.5 0.5 0.5" mass="0"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"""

earth = SpaceAGORA.make_no_gram_planet(:earth)
gravity = (SpaceAGORA.InverseSquaredGravityModel(),)
r1 = SVector(7.0e6, 0.0, 0.0)
v1 = SVector(0.0, sqrt(earth.μ / norm(r1)), 0.0)
r2 = r1 + SVector(0.0, 20.0, 0.0)
v2 = v1
initial_states = [SceneBodyState("chaser", r1, v1),
                  SceneBodyState("target", r2, v2)]

scene = ProximityScene(;
    mjcf_xml=xml, dt=0.02, planet=earth,
    gravity_effectors=gravity, initial_states=initial_states)

for _ in 1:500
    scene_step!(scene)
end

chaser = scene_body_state(scene, "chaser")
target = scene_body_state(scene, "target")
println("Elapsed time: ", scene_time(scene), " s")
println("Separation: ", norm(target.r - chaser.r), " m")
println("Relative velocity: ", target.v - chaser.v, " m/s")
```

Each cube has a one-metre side, mass 12 kg and diagonal inertia 2 kg m². The
explicit inertial elements supply the mass properties; the zero-mass geometries
supply shape only. Contact is disabled here to isolate orbital motion. Each
independently flying root has exactly one free joint and one initial state.
Articulated child bodies may have their own joints, but bodies fixed to the
MuJoCo world are not supported by this runner.

The `dt` argument is required and controls the scene step even if the MJCF file
contains an `<option timestep="...">` setting. This example uses only point-mass
gravity and the data-free Earth constructor. It needs neither native GRAM data
nor a planetary rotation callback.

## Choosing gravity and numerical resolution

`gravity_effectors` must be a nonempty tuple of position-only static gravity
models supported by SpaceAGORA's gravity interface. J2 and spherical-harmonic
models also require `planet_rotation(t)`, returning an
`SMatrix{3,3,Float64}` that maps inertial vectors into planet-fixed axes. The
callback receives **scene time**, not a calendar date. Define its epoch and time
scale consistently with the spacecraft initial state and the chosen ephemeris
model. In a coupled run, use `t0=0.0` and make elapsed time zero correspond to the
configuration's `initial_time`. Merely selecting the same planet does not align
two independently defined rotation callbacks.

Gravity is applied as a force at each MuJoCo body's center of mass. For an
articulated structure, forces on its distinct bodies can produce moments about
the joints. This interface does not add an intrinsic gravity-gradient torque
for the finite extent of a single rigid body.

The complete scene method is first order in `dt`: gravitational wrenches are
held during each native step. The chief's separate RK4 update does not make the
coupled method fourth order. Supported native choices are `:implicitfast` (the
default), `:implicit` and `:euler`. `:rk4` is rejected because MuJoCo's split-step
path does not perform RK4 integration.

Choose `dt` from the observables that matter to the study. Repeat a case at
smaller steps and compare relative position, attitude, joint motion and contact
response as appropriate. Tightening the SpaceAGORA ODE tolerances cannot remove
the scene's fixed-step error. The [architecture note](architecture.md#measured-agreement)
records the measured orbital convergence and the limitations observed in
attitude tests; those measurements are not error bounds for a new scene.

## Resetting, branching and applying forces

A scene is mutable. Step a given instance from one task at a time; independent
copies have separate native models and data. To branch the standalone example
at its current state:

```julia
saved = scene_state(scene)
branch = copy(scene)
scene_step!(branch)                 # leaves scene unchanged
scene_step!(scene)
scene_set_state!(scene, saved)      # restores the saved scene state
scene_reset!(scene)                 # returns to its stored initial states
```

A `SceneState` contains the chief state, step counter and native integration
state, including actuator and warm-start data. Restore it only into a scene with
the same model definition, step, epoch and gravity settings; it is not a general
interchange format between MJCF models or native library versions.
`scene_reset!(scene, new_states)` also replaces the
stored initial states used by subsequent resets. Each copy owns separate native
memory; close the copies you no longer need (see [Native memory](#native-memory)).

For a standalone non-gravitational force, construct a matrix with one column per
entry in `scene_body_names(scene)`, including articulated children. For example,
after the reset above:

```julia
body_names = scene_body_names(scene)
forces = zeros(3, length(body_names))
chaser_column = only(findall(==("chaser"), body_names))
forces[:, chaser_column] .= (0.01, 0.0, 0.0)
scene_step!(scene; external_forces=forces)
```

The force is applied for that one step. Its net contribution also accelerates
the chief, so a common acceleration is represented consistently in the absolute
states. There is no external-torque keyword. MJCF actuator commands can be set
through the mutable vector returned by `scene_ctrl(scene)`; their interpretation
depends on the actuators defined in the model. The two-cube example has none.
These standalone facilities do not connect SpaceAGORA control effectors to a
coupled scene.

## Carrying the scene inside SpaceAGORA

The coupling assigns a scene exclusive propagation ownership of selected
spacecraft for a run. A pair such as `1 => "chaser"` refers to the **position in
the configuration's spacecraft vector**, not the spacecraft's `id`. Every free
root in that scene must be paired once. A child link belongs to its MuJoCo
assembly and is not assigned a separate spacecraft entry.

The following continuation of `two_cubes.jl` builds a complete configuration
through public constructors. The core spacecraft descriptions have the same
mass, dimensions and initial states as the MJCF cubes. Both bodies are owned by
the scene; ordinary core-propagated spacecraft could be included in the same
vector without assigning them to this owner.

```julia
EP = SpaceAGORA.ExternalPropagationServices
EP.require_version(v"2.0.0")

function cube_spacecraft(r, v, id)
    bus = SpaceAGORA.Link(;
        root=true, m=12.0, dims=MVector(1.0, 1.0, 1.0))
    return SpaceAGORA.SpacecraftModel(;
        joints=SpaceAGORA.Joint[], links=[bus], root=bus,
        inertia_tensor=bus.inertia,
        initial_condition=SpaceAGORA.CartesianInitialCondition(r, v),
        id=id)
end
spacecraft = [cube_spacecraft(r1, v1, 1), cube_spacecraft(r2, v2, 2)]

config = SpaceAGORA.SimulationConfiguration(;
    simulation_settings=SpaceAGORA.SimulationSettings(;
        results=false, verbose=false, generate_plots=false),
    mission_configuration=SpaceAGORA.MissionConfiguration(;
        mission_type=SpaceAGORA.MissionTime, mission_time=10.0,
        keplerian=true, orientation_sim=true, data_rate=1.0),
    environment_model=SpaceAGORA.EnvironmentModel(;
        planet=earth, EI=300.0, density_model=SpaceAGORA.NoAtmosphereModel(),
        ephemerides_model=SpaceAGORA.SimpleEphemeridesModel(),
        thermal_model=SpaceAGORA.MaxwellianHeat(;
            thermal_accomodation_factor=1.0, planet=earth),
        topography=false, wind=false),
    dynamics_model=SpaceAGORA.DynamicsModel(spacecraft, gravity),
    initial_time=SpaceAGORA.InitialTime(;
        year=2026, month=1, day=1, hour=0, minute=0, second=0.0),
    solver_config=SpaceAGORA.SolverConfig(; solver_mode=:dp8))

template = ProximityScene(;
    mjcf_xml=xml, dt=0.02, planet=earth,
    gravity_effectors=gravity, initial_states=initial_states)
owner = ProximitySceneDynamics(template, 1 => "chaser", 2 => "target";
                              mass_rtol=1e-12)
config = EP.with_external_propagators(config, (owner,))
EP.validate_external_propagators(config, :dp8)

fields = vcat(SpaceAGORA.default_save_fields(config),
              [scene_body_pose_save_field("chaser"),
               scene_body_pose_save_field("target")])
result = SpaceAGORA.run_simulation(config; return_results=true, save_fields=fields)
table = result.table
println("Saved rows: ", size(table, 1))
println("Template time after the run: ", scene_time(template), " s")
```

The template remains at its initial time. Each run prepares a separate native
scene and resets it from the paired **core spacecraft initial conditions**;
the template's `initial_states` do not override those conditions. Enable
`orientation_sim` when carrying initial attitude and body rate into the scene.
With orientation simulation disabled, the core supplies identity attitude and
zero angular velocity to the owner.

The scene's planet and gravity effectors govern its bodies. Core dynamic
effectors do not additionally act on owned spacecraft, so keep the two
descriptions consistent rather than expecting core settings to replace the
scene's settings. The optional `mass_rtol` check compares each paired spacecraft's
total mass with its MJCF root body's mass; it does not sum an articulated
subtree or check its inertia. The check is appropriate for the single-body
cubes above. More complex assemblies need an explicit mass-property review.

`with_external_propagators` returns a shallowly updated configuration and leaves
the original unchanged. `validate_external_propagators` provides an early check
of ownership, the stated solver mode and configuration restrictions. Run setup
still performs routing and native preparation checks. Users of this adapter do
not need to implement the service hooks or access engine storage. Authors of
other adapters should use the
[versioned service contract](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/bec28fcc348b48c6e807d007f9a5009e253e8340/docs/src/reference/external_propagation_services.md).

## Reading coupled output

The core solver advances adaptively while the scene advances in whole `dt`
steps. After an accepted core step, the engine synchronizes the external scene
before other discrete callbacks. When core time lies between scene steps, the
spacecraft entries use a short extrapolation based on chief acceleration and
constant body angular rate. At a scene boundary they contain the native scene
state. Guidance and navigation periods must be integer multiples of `dt` so
their callbacks can read synchronized states at their ticks.

Ordinary saved spacecraft columns can include the core solver's dense-output
interpolation. Setting `data_rate` to a multiple of `dt` alone does not force
every saved row to be an exact native state. Do not use row-for-row identity
with a standalone run as a default expectation.

The added fields `scene_pose_chaser_1` through `scene_pose_chaser_7` contain
three absolute inertial COM coordinates followed by a **scalar-last** SpaceAGORA
quaternion (`v_body = rot(q) * v_inertial`).
They read the current last completed native state when the saving callback
executes, rather than the off-step spacecraft extrapolation. If the solver fills
an earlier output row after a larger accepted step, the native pose can be later
than that row's timestamp. The getter does not reconstruct a historical pose.
For a time-matched comparison, force accepted steps at the requested native
boundaries with an aligned periodic callback, as the coupling tests do; merely
setting `data_rate` is insufficient. The same helper can save an articulated
child body's pose. Use distinct body names across multiple scene owners when
selecting bodies this way.

## Native memory

Each scene owns a MuJoCo model and data in native memory that Julia's garbage
collector does not see. A loop that drops scenes, such as discarded `copy`
branches or rebuilt episodes, therefore keeps their native memory until a
collection happens to run, and the loop itself may never prompt one. Call
`close(scene)` when a scene is no longer needed. Continuing `two_cubes.jl`:

```julia
separations = Float64[]
for thrust in (0.005, 0.01, 0.02)
    branch = copy(scene)
    kick = zeros(3, length(body_names))
    kick[1, chaser_column] = thrust
    try
        for _ in 1:500
            scene_step!(branch; external_forces=kick)
        end
        a = scene_body_state(branch, "chaser")
        b = scene_body_state(branch, "target")
        push!(separations, norm(b.r - a.r))
    finally
        close(branch)
    end
end
```

`close` is idempotent. Afterwards, stepping, resetting, copying, reading body
states or saving the state of that scene throws an `InvalidStateException`;
`scene_time`, `scene_chief` and `scene_body_names` still return the last values.
A vector obtained earlier from `scene_ctrl` points into the freed memory and must
not be used. Closing a scene does not affect its copies. Do not close a scene
while another task is stepping it. A scene that is never closed is released by
its finalizer once the garbage collector reclaims it.

Most of a scene's native memory is the MuJoCo arena of its data: a fixed block
that holds MuJoCo's stack and its contact and constraint arrays. Without a
`<size memory="..."/>` element MuJoCo reserves a default sized for general
models, far larger than a small proximity scene uses. A smaller arena is set in
the MJCF, and `copy` keeps it:

```xml
<mujoco>
  <size memory="1M"/>
  <worldbody>
    ...
  </worldbody>
</mujoco>
```

Size it from a measurement. `scene_arena_use(scene)` returns the arena `size` and
the `peak` use since the scene was built, copied or last reset, both in bytes.
Run a representative worst case, with the most contacts the study expects, and
leave a generous margin above the peak. **An arena too small for a step
terminates the Julia process.** MuJoCo reports the overflow through its error
handler, which exits the process, and this package does not replace that handler
because it is shared by everything in the process. The default arena has the
same limit, which a scene with very many contacts can reach. An arena too small
for the model itself is reported as an ordinary error when the model is
compiled.

Inside SpaceAGORA, every `run_simulation` copies the template scene, and the
garbage collector releases that copy after the run; the coupling has no hook to
close it when the run ends. In long or threaded Monte Carlo campaigns, a
template with a measured `<size memory="..."/>` keeps those pending copies
small. Closing the template does not affect runs that have already copied it.
A closed template is refused at setup.

## Scope and practical limits

Coupled propagation currently supports solver modes `:tsit5`, `:auto_stiff`,
`:rodas5p` and `:dp8`. Its mechanics remain in the scene: native joints and
contacts can act, but core aerodynamic, solar-pressure and control loads are
not transferred to owned bodies. Core control effectors on those bodies are
refused. A spacecraft cannot simultaneously use scene ownership and the core's
articulated joints, compliant attachments or robot-arm dynamics.

In particular, this adapter does not feed MuJoCo contact forces back into
SpaceAGORA's compliant multibody model. Keeping that model while adding MuJoCo
contacts needs a two-way coupling layer that synchronizes positions and
velocities, transfers contact forces and moments, and validates stability and
energy behavior. This work is **not implemented or validated**; the development
plan is tracked in [issue #4](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl/issues/4).

Use a fixed mission duration. Orbit-count termination, atmosphere-interface
events and continuous touchdown events on owned spacecraft are outside the
supported coupling. Core impact detection does not monitor the owned spacecraft
entries; MuJoCo contacts concern the geometry actually present in the scene,
not an implicit planetary surface.

Standalone snapshots do not enable core checkpoint/resume, which is refused
for external owners. Threaded Monte Carlo runs use separate native scene copies
(see [Native memory](#native-memory));
process-pool campaigns and `run_constellation_ensemble` are unsupported because
native state is not a serializable worker payload. The flat RHS route is also
refused. If the runner reports a native numerical-instability reset, treat that
trajectory as failed and revise the model or step size before restarting it.

The [scene tests](../test/scene_tests.jl) exercise orbital agreement, state
restoration and standalone forcing. The [coupling tests](../test/engine_tests.jl)
cover synchronization, attitude conventions, ownership and refusal cases.
For the implementation, see the [scene runner](../src/proximity_scene.jl) and
[core adapter](../src/engine_coupling.jl). The
[Basilisk comparison study](../benchmarks/basilisk_mujoco_xval/README.md) documents
its cases and unresolved limits; it does not establish universal agreement for
contact or tumbling dynamics. These sources support the current interface and
its demonstrated cases, while each new scientific application still needs its
own resolution and model checks.
