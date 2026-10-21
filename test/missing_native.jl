# Run in a fresh process with the installed environment. Without the MuJoCo
# library, both packages still load, and a scene is refused with a clear error
# before any native code loads. On Linux x86_64 this test hides the pinned artifact
# behind an empty override in a temporary depot; on other platforms the package
# has no library to load.
using Test, TOML

artifact = TOML.parsefile(joinpath(@__DIR__, "..", "Artifacts.toml"))["mujoco_3_11_0_linux_x86_64"]
expected = Sys.islinux() && Sys.ARCH === :x86_64 ? "libmujoco.so.* not found" : "unsupported platform"

const CHILD = """
using SpaceAGORA, SpaceAGORAMuJoCo, StaticArrays, Libdl
native() = any(path -> occursin(r"(?i)^(lib)?mujoco([.-]|\$)", basename(path)), Libdl.dllist())
native() && error("the MuJoCo library loaded with the package")
xml = "<mujoco><worldbody><body name='chaser'><freejoint/><inertial pos='0 0 0' mass='20' diaginertia='2 3 4'/></body></worldbody></mujoco>"
r = SVector(7.0e6, 0.0, 0.0)
v = SVector(0.0, sqrt(3.986004418e14 / 7.0e6), 0.0)
problem = try
    ProximityScene(; mjcf_xml=xml, dt=0.05, planet=SpaceAGORA.make_no_gram_planet(:earth),
        gravity_effectors=(SpaceAGORA.InverseSquaredGravityModel(),),
        initial_states=[SceneBodyState("chaser", r, v)])
    nothing
catch error
    error
end
problem === nothing && error("a scene was created without the MuJoCo library")
native() && error("the MuJoCo library loaded after the refusal")
println("REFUSED: ", sprint(showerror, problem))
"""

@testset "A scene without the MuJoCo library is refused" begin
    mktempdir() do depot
        hidden = mkpath(joinpath(depot, "empty-artifact"))
        mkpath(joinpath(depot, "artifacts"))
        open(joinpath(depot, "artifacts", "Overrides.toml"), "w") do io
            TOML.print(io, Dict(artifact["git-tree-sha1"] => hidden))
        end
        command = `$(Base.julia_cmd()) --startup-file=no --project=$(Base.active_project()) -e $CHILD`
        output = read(addenv(command, "JULIA_DEPOT_PATH" => join([depot; DEPOT_PATH], Sys.iswindows() ? ';' : ':')), String)
        print(output)
        @test occursin("REFUSED: ", output)
        @test occursin(expected, output)
    end
end
