"""
Optional MuJoCo proximity scenes for SpaceAGORA: rigid-body contact-capable multibody scenes (MJCF) that
fly in a chief-centered inertial frame while SpaceAGORA's own gravity models drive the chief orbit and the
per-body differential gravity.

Load with `using SpaceAGORAMuJoCo`. The native library is MuJoCo 3.11.0 from a lazy artifact (Linux x86_64);
see `README.md` for the dependency review. Stage 1 is the standalone scene runner: [`ProximityScene`](@ref),
[`scene_reset!`](@ref), [`scene_step!`](@ref), [`scene_state`](@ref), [`scene_set_state!`](@ref), and `close(scene)`
to free its native memory. Stage 2 runs a scene inside `run_simulation` through [`ProximitySceneDynamics`](@ref).
"""
module SpaceAGORAMuJoCo

import SpaceAGORA
isdefined(SpaceAGORA, :ExternalPropagationServices) || throw(ArgumentError(
    "This SpaceAGORA revision has no ExternalPropagationServices contract. Install the core revision pinned by SpaceAGORAMuJoCo/scripts/setup_env.jl."))
const EP = SpaceAGORA.ExternalPropagationServices
const CORE_CONTRACT_VERSION = v"1.0.0"
EP.require_version(CORE_CONTRACT_VERSION)

# Recheck when loading a precompiled companion against a different core checkout.
function __init__()
    EP.require_version(CORE_CONTRACT_VERSION)
end

include("binding.jl")
include("proximity_scene.jl")
include("engine_coupling.jl")

export ProximityScene, SceneBodyState, SceneState, body_state_from_initial_condition
export sa_to_mujoco_quaternion, mujoco_to_sa_quaternion, ProximitySceneDynamics, ProximitySceneRuntime, scene_body_pose_save_field
export scene_reset!, scene_step!, scene_state, scene_set_state!, scene_time, scene_chief, scene_body_names,
    scene_body_state, scene_ctrl, scene_arena_use

end # module SpaceAGORAMuJoCo
