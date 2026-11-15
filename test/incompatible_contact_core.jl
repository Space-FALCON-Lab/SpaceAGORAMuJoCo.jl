# A fresh-process check of the actual entrypoint's contact version refusal.
using Test, Libdl, SpaceAGORA
isdefined(SpaceAGORA,:ContactLoadServices) || error("Paired-core CI must provide ContactLoadServices")
@testset "contact mismatch refuses before any adapter definitions" begin
    path=joinpath(@__DIR__,"..","src","SpaceAGORAMuJoCo.jl")
    source=read(path,String)
    # Replace just the contract include with the same source requesting 0.0.0.
    contract=read(joinpath(dirname(path),"contact_contract.jl"),String)
    @test count("v\"0.2.0\"",contract)==1
    incompatible=replace(source,"include(\"contact_contract.jl\")"=>replace(contract,"v\"0.2.0\""=>"v\"0.0.0\""))
    failure=try
        Base.include_string(Main,incompatible,path)
        nothing
    catch err
        while err isa LoadError
            err=err.error
        end
        err
    end
    @test failure isa SpaceAGORA.ContactLoadServices.CompatibilityError
    @test !isdefined(Main.SpaceAGORAMuJoCo,:Binding)
    @test !isdefined(Main.SpaceAGORAMuJoCo,:BusBumperContact)
    @test !isdefined(Main.SpaceAGORAMuJoCo,:ProximitySceneDynamics)
    @test !any(p->occursin(r"(?i)^(lib)?mujoco([.-]|$)",basename(p)),Libdl.dllist())
end
