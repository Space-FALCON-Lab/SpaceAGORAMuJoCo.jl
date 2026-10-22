# Definitions shared by the scene and engine tests (included once by runtests.jl).
using LinearAlgebra
using SpaceAGORAMuJoCo: Binding
using StaticArrays
import SpaceAGORA
using SpaceAGORA: make_example_config

# The engine tests reach three SpaceAGORA internals that have no public name: the
# periodic callback that aligns saves, quaternion rotation and unit-quaternion
# projection. Everything else uses SpaceAGORA's public names.
const SM = SpaceAGORA.SimulationModel
const EP = SpaceAGORA.ExternalPropagationServices

# Fixture-only reconstruction using the public configuration constructor. Preserve
# every field/reference and infer model parameters again, as in the original tests.
function fixture_configuration(args; overrides...)
    names = fieldnames(typeof(args))
    fields = NamedTuple{names}(map(name -> getfield(args, name), names))
    return SpaceAGORA.SimulationConfiguration(; merge(fields, (; overrides...))...)
end
const MU = 3.986004418e14

# Two free rigid boxes, body frames at their centers of mass, no contact, no joints.
const TWO_BODIES = """
<mujoco>
  <compiler angle="degree"/>
  <option timestep="0.05"/>
  <worldbody>
    <body name="chaser"><freejoint/>
      <inertial pos="0 0 0" mass="20" diaginertia="2 3 4"/>
      <geom type="box" size="0.5 0.4 0.3" contype="0" conaffinity="0" mass="0"/>
    </body>
    <body name="target"><freejoint/>
      <inertial pos="0 0 0" mass="35" diaginertia="5 6 7"/>
      <geom type="box" size="0.6 0.6 0.6" contype="0" conaffinity="0" mass="0"/>
    </body>
  </worldbody>
</mujoco>"""

const EARTH = SpaceAGORA.make_no_gram_planet(:earth)
const EPH = SpaceAGORA.SimpleEphemeridesModel()
const T0 = SpaceAGORA.InitialTime(year=2014, month=5, day=27, hour=5, minute=0, second=0.0)
const ET0 = SpaceAGORA.ephemerides_epoch_seconds(EPH, T0)
planet_rotation(t) = SpaceAGORA.ephemerides_frame_lpi(EPH, EARTH, ET0 + t)

# chaser on a 51.6 deg circular orbit at r = 7000 km; the target is ~50 m away with a small velocity offset.
const R1 = SVector(7.0e6, 0.0, 0.0)
const V1 = sqrt(MU / 7.0e6) .* SVector(0.0, cosd(51.6), sind(51.6))
const R2 = R1 + SVector(30.0, -40.0, 15.0)
const V2 = V1 + SVector(0.05, 0.02, -0.03)
# Tolerances at dt = 0.0125 s, set from the measured first-order convergence (errors scale as dt; measured
# 1.70e-3 m, 2.17e-6 m/s and 2.68e-3 m over one orbit, point mass and J2), with 1.5x margin.
const TOL_POSITION_M = 2.5e-3
const TOL_VELOCITY_MPS = 3.3e-6
const TOL_RELATIVE_POSITION_M = 4.0e-3
const PERIOD = 2π * sqrt(7.0e6^3 / MU)

states() = [SceneBodyState("chaser", R1, V1), SceneBodyState("target", R2, V2)]
make_scene(effectors; dt, kwargs...) = ProximityScene(; mjcf_xml=TWO_BODIES, dt, planet=EARTH, gravity_effectors=effectors,
    initial_states=states(), planet_rotation=planet_rotation, kwargs...)

# --- the same two bodies as ordinary SpaceAGORA spacecraft ---------------------------------------------

mklink(; root=false, m, dims, r=(0.0, 0.0, 0.0)) = SpaceAGORA.Link(root=root, m=m, dims=MVector{3, Float64}(dims...), r=MVector{3, Float64}(r...), q=MVector{4, Float64}(0, 0, 0, 1))
# Spacecraft that run together need distinct ids; the tests number them by position.
function rigid_sc(m, dims, r, v; id=1)
    bus = mklink(root=true, m=m, dims=dims)
    ic = SpaceAGORA.CartesianInitialCondition(r, v)
    return SpaceAGORA.SpacecraftModel(; joints=SpaceAGORA.Joint[], links=[bus], root=bus, initial_condition=ic, inertia_tensor=bus.inertia, id)
end
function engine_reference(effectors, tend, data_rate)
    tol = SpaceAGORA.IntegrationTolerances(reltol_orbit=1e-12, abstol_orbit=1e-9, reltol_atmosphere=1e-12, abstol_atmosphere=1e-9,
        reltol_quaternion=1e-12, abstol_quaternion=1e-9, reltol_mass=1e-12, abstol_mass=1e-9,
        reltol_angular_rate=1e-12, abstol_angular_rate=1e-9, dt_max_orbit=0.5, dt_max_atmosphere=0.5)
    scs = SpaceAGORA.SpacecraftModel[rigid_sc(20.0, (1.0, 0.8, 0.6), R1, V1; id=1), rigid_sc(35.0, (1.2, 1.2, 1.2), R2, V2; id=2)]
    base = make_example_config(planet=EARTH, spacecraft=scs[1], mission_time=tend, initial_time=T0,
        dynamic_effectors=effectors, density_model=SpaceAGORA.NoAtmosphereModel(), ephemerides_model=EPH,
        orientation_sim=false, keplerian=true, verbose=false, results=false, results_directory=mktempdir(),
        solver_config=SpaceAGORA.SolverConfig(solver_mode=:dp8))
    args = fixture_configuration(base;
        integration_tolerances=tol,
        mission_configuration=SpaceAGORA.MissionConfiguration(
            mission_type=base.mission_configuration.mission_type, keplerian=true, number_of_orbits=1,
            mission_time=tend, orientation_sim=false, num_steps_to_save=100000, data_rate=data_rate),
        dynamics_model=SpaceAGORA.DynamicsModel(scs, effectors))
    return SpaceAGORA.run_simulation(args; return_results=true).table
end


col(df, k) = [df[!, "sc1_pos_$k"], df[!, "sc1_vel_$k"], df[!, "sc2_pos_$k"], df[!, "sc2_vel_$k"]]
function samples_from_table(df, every, tend)
    t = Float64.(df[!, :time])
    idx = [findmin(abs.(t .- s))[2] for s in every:every:tend]
    all(isapprox.(t[idx], collect(every:every:tend); atol=1e-6)) || error("engine output times do not line up")
    c = [col(df, k) for k in 1:3]
    return [(SVector(c[1][1][i], c[2][1][i], c[3][1][i]), SVector(c[1][2][i], c[2][2][i], c[3][2][i]),
             SVector(c[1][3][i], c[2][3][i], c[3][3][i]), SVector(c[1][4][i], c[2][4][i], c[3][4][i])) for i in idx]
end

function max_differences(a, b)
    dr = maximum(norm(x[1] - y[1]) for (x, y) in zip(a, b)); dv = maximum(norm(x[2] - y[2]) for (x, y) in zip(a, b))
    dr2 = maximum(norm(x[3] - y[3]) for (x, y) in zip(a, b))
    drel = maximum(norm((x[3] - x[1]) - (y[3] - y[1])) for (x, y) in zip(a, b))
    dvrel = maximum(norm((x[4] - x[2]) - (y[4] - y[2])) for (x, y) in zip(a, b))
    return (dr = dr, dv = dv, dr_target = dr2, drel = drel, dvrel = dvrel)
end


# Run the scene to `tend`, sampling absolute states every `every` seconds (a whole number of steps).
function scene_samples(effectors, dt, tend, every)
    sc = make_scene(effectors; dt)
    per = round(Int, every / dt); @assert per * dt ≈ every
    nsteps = round(Int, tend / dt)
    out = Vector{NTuple{4, SVector{3, Float64}}}()
    for k in 1:nsteps
        scene_step!(sc)
        if k % per == 0
            a = scene_body_state(sc, "chaser"); b = scene_body_state(sc, "target")
            push!(out, (a.r, a.v, b.r, b.v))
        end
    end
    return out
end
