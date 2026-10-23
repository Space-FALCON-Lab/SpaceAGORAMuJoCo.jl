# This file uses only standard libraries and does not call Pkg installation APIs.
using Test, TOML
include(joinpath(@__DIR__, "..", "scripts", "setup_env.jl"))
using .CompanionInstallation

@testset "independent companion installation plan" begin
    package = dirname(@__DIR__)
    project = TOML.parsefile(joinpath(package, "Project.toml"))
    pin = TOML.parsefile(joinpath(package, "SpaceAGORASource.toml"))
    @test pin["uuid"] == project["deps"]["SpaceAGORA"]
    @test !haskey(get(project, "sources", Dict()), "SpaceAGORA")
    @test pin["version"] == "0.6.0"
    @test project["uuid"] == "a46e04e1-adb7-43c3-a03b-26575bf4fb67"
    @test project["compat"]["SpaceAGORA"] == "0.5, 0.6, 0.7"  # outer families only; the exact pin and contract are required
    @testset "declared standard-library identities" begin
        # Read this Julia installation's identifiers, rather than maintaining a
        # second handwritten UUID table that could repeat a metadata typo.
        stdlibs = Dict(name => string(uuid) for (uuid, (name, _)) in CompanionInstallation.Pkg.Types.stdlibs())
        @test stdlibs["Test"] == string(Base.PkgId(Test).uuid)
        for section in ("deps", "extras"), (name, uuid) in sort!(collect(project[section]))
            haskey(stdlibs, name) || continue
            @testset "$section.$name" begin
                @test uuid == stdlibs[name]
            end
        end
    end
    mktempdir() do directory
        env = joinpath(directory, "environment")
        default = CompanionInstallation.installation_plan(env; core_path="", core_revision="")
        @test default.core.revision == pin["rev"]
        @test default.core.url == pin["url"]
        @test default.core.path === nothing
        @test !ispath(env)  # planning never activates or creates an environment
        @test Set(keys(default.dependencies)) == union(setdiff(Set(keys(project["deps"])), ["SpaceAGORA"]), Set(keys(project["extras"])))
        override = repeat("a", 40)
        @test CompanionInstallation.installation_plan(env; core_path="", core_revision=override).core.revision == override
        @test_throws ArgumentError CompanionInstallation.installation_plan(env; core_path="", core_revision="main")
        @test_throws ArgumentError CompanionInstallation.installation_plan(package; core_path="", core_revision="")
        @test_throws ArgumentError CompanionInstallation.installation_plan(env; core_path=directory, core_revision="")
        core = mkpath(joinpath(directory, "core"))
        write(joinpath(core, "Project.toml"), "name = \"WrongPackage\"\nuuid = \"$(project["deps"]["SpaceAGORA"])\"\n")
        @test_throws ArgumentError CompanionInstallation.installation_plan(env; core_path=core, core_revision="")
        write(joinpath(core, "Project.toml"), "name = \"SpaceAGORA\"\nuuid = \"$(project["deps"]["SpaceAGORA"])\"\n")
        local_plan = CompanionInstallation.installation_plan(env; core_path=core, core_revision="")
        @test local_plan.core.path == realpath(core)
        @test local_plan.core.url === nothing && local_plan.core.revision === nothing
        @test_throws ArgumentError CompanionInstallation.installation_plan(env; core_path=core, core_revision=override)
        @test_throws ArgumentError CompanionInstallation.installation_plan(core; core_path=core, core_revision="")
        @test !ispath(env)
    end
end
