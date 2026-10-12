# Explicitly install the pinned core and this companion into a separate environment.
# No parent checkout, parent Manifest.toml, HYPR setup or native GRAM data is used.
#   julia --startup-file=no scripts/setup_env.jl <environment-directory>
module CompanionInstallation

using Pkg
using TOML

const PACKAGE_ROOT = realpath(joinpath(@__DIR__, ".."))

function installation_plan(directory::AbstractString;
        core_path::AbstractString=get(ENV, "SPACEAGORA_CORE_PATH", ""),
        core_revision::AbstractString=get(ENV, "SPACEAGORA_CORE_REV", ""))
    project = TOML.parsefile(joinpath(PACKAGE_ROOT, "Project.toml"))
    destination = ispath(directory) ? realpath(directory) : abspath(directory)
    destination == PACKAGE_ROOT && throw(ArgumentError("Choose a separate environment directory, not the companion checkout"))
    isempty(core_path) || isempty(core_revision) || throw(ArgumentError("Set only one of SPACEAGORA_CORE_PATH and SPACEAGORA_CORE_REV"))
    pin = project["sources"]["SpaceAGORA"]
    if isempty(core_path)
        revision = isempty(core_revision) ? pin["rev"] : String(core_revision)
        occursin(r"^[0-9a-fA-F]{40}$", revision) || throw(ArgumentError("SpaceAGORA must be pinned to a full 40-character commit SHA"))
        core = (; path=nothing, url=pin["url"], revision)
    else
        path = abspath(core_path)
        isfile(joinpath(path, "Project.toml")) || throw(ArgumentError("SPACEAGORA_CORE_PATH must contain a Project.toml"))
        path = realpath(path)
        core_project = TOML.parsefile(joinpath(path, "Project.toml"))
        get(core_project, "name", "") == "SpaceAGORA" &&
            get(core_project, "uuid", "") == project["deps"]["SpaceAGORA"] ||
            throw(ArgumentError("SPACEAGORA_CORE_PATH is not the SpaceAGORA package"))
        destination == path && throw(ArgumentError("Choose an environment directory separate from the core checkout"))
        core = (; path, url=nothing, revision=nothing)
    end
    # Direct test and benchmark imports must resolve from the independent project,
    # with JULIA_LOAD_PATH=@:@stdlib and without an ambient developer environment.
    dependencies = merge(project["deps"], project["extras"])
    delete!(dependencies, "SpaceAGORA")
    return (; destination, core, dependencies, companion=PACKAGE_ROOT)
end

function setup(plan; precompile::Bool=true)
    previous = Base.active_project()
    try
        withenv("JULIA_PKG_PRECOMPILE_AUTO" => "0") do
            Pkg.activate(plan.destination)
            # Explicit core installation is intentional: dependency [sources]
            # propagation is not assumed, including on Julia 1.12.
            if plan.core.path === nothing
                Pkg.add(Pkg.PackageSpec(url=plan.core.url, rev=plan.core.revision))
            else
                Pkg.develop(Pkg.PackageSpec(path=plan.core.path))
            end
            Pkg.develop(Pkg.PackageSpec(path=plan.companion); preserve=Pkg.PRESERVE_ALL)
            direct = [Pkg.PackageSpec(name=name, uuid=uuid) for (name, uuid) in sort!(collect(plan.dependencies))]
            Pkg.add(direct; preserve=Pkg.PRESERVE_ALL)
            Pkg.instantiate()
            precompile && Pkg.precompile(["SpaceAGORA", "SpaceAGORAMuJoCo"]; strict=true)
        end
    finally
        previous === nothing ? Pkg.activate() : Pkg.activate(dirname(previous))
    end
    return plan.destination
end

function main(args=ARGS)
    length(args) == 1 || throw(ArgumentError("Usage: julia scripts/setup_env.jl <environment-directory>"))
    plan = installation_plan(only(args))
    println("Companion: ", plan.companion)
    println("Core: ", plan.core.path === nothing ? "$(plan.core.url) @ $(plan.core.revision)" : plan.core.path)
    return setup(plan)
end

end

abspath(PROGRAM_FILE) == abspath(@__FILE__) && CompanionInstallation.main()
