# Exercise the actual entrypoint with a deliberately incompatible requested
# contract: the previous contract, 1.0.0, so a pairing across the 2.0.0 bump is
# refused. Only the expected-version constant changes in this test copy.
# Run in a fresh process without importing SpaceAGORAMuJoCo first.
using Test, Libdl
using SpaceAGORA

@testset "contract mismatch refuses before native binding or adapter definitions" begin
    @test !isdefined(Main, :SpaceAGORAMuJoCo)
    path = joinpath(@__DIR__, "..", "src", "SpaceAGORAMuJoCo.jl")
    source = read(path, String)
    expected = "const CORE_CONTRACT_VERSION = v\"2.0.0\""
    @test count(expected, source) == 1
    incompatible = replace(source, expected => "const CORE_CONTRACT_VERSION = v\"1.0.0\"")
    failure = try
        Base.include_string(Main, incompatible, path)
        nothing
    catch error
        while error isa LoadError
            error = error.error
        end
        error
    end
    @test failure isa SpaceAGORA.ExternalPropagationServices.CompatibilityError
    message = sprint(showerror, failure)
    @test occursin("1.0.0", message) && occursin("2.0.0", message)
    @test isdefined(Main, :SpaceAGORAMuJoCo)
    @test !isdefined(Main.SpaceAGORAMuJoCo, :Binding)
    @test !isdefined(Main.SpaceAGORAMuJoCo, :ProximitySceneDynamics)
    @test !any(path -> occursin(r"(?i)^(lib)?mujoco([.-]|$)", basename(path)), Libdl.dllist())
    for name in SpaceAGORA.ExternalPropagationServices.IMPLEMENTED_FUNCTIONS
        @test !any(method -> method.module === Main.SpaceAGORAMuJoCo,
            methods(getproperty(SpaceAGORA.ExternalPropagationServices, name)))
    end
end
