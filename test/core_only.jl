# Run in a fresh process: a core import and failed compatibility request must
# neither import the companion nor load libmujoco.
using Test, Libdl
using SpaceAGORA

native_loaded() = any(path -> occursin(r"(?i)^(lib)?mujoco([.-]|$)", basename(path)), Libdl.dllist())
companion_loaded() = any(pkg -> pkg.name == "SpaceAGORAMuJoCo", keys(Base.loaded_modules))

@testset "core-only adapter isolation" begin
    @test !companion_loaded()
    @test !native_loaded()
    services = SpaceAGORA.ExternalPropagationServices
    @test_throws services.CompatibilityError services.require_version(v"0.0.0")
    @test !companion_loaded()
    @test !native_loaded()
end
