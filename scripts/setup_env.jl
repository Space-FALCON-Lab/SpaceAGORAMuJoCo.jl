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
        revision = lowercase(isempty(core_revision) ? pin["rev"] : String(core_revision))
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

# Use a working-tree checkout rather than Pkg's bare LibGit2 extraction, which
# cannot materialize this core revision's uninitialized submodule entries.
function source_git(args...)
    command = Cmd(["git", "-c", "submodule.recurse=false", "-c", "filter.lfs.process=",
        "-c", "filter.lfs.smudge=", "-c", "filter.lfs.clean=", "-c", "filter.lfs.required=false",
        String.(args)...])
    return addenv(command, "GIT_LFS_SKIP_SMUDGE" => "1", "GIT_DIR" => nothing,
        "GIT_WORK_TREE" => nothing, "GIT_INDEX_FILE" => nothing, "GIT_COMMON_DIR" => nothing,
        "GIT_OBJECT_DIRECTORY" => nothing, "GIT_ALTERNATE_OBJECT_DIRECTORIES" => nothing)
end

function verify_core_checkout(path, core)
    islink(path) && throw(ArgumentError("Refusing a symlinked managed core checkout: $path"))
    gitdir = joinpath(path, ".git")
    isdir(gitdir) && !islink(gitdir) || throw(ArgumentError("Not a setup-owned core checkout: $path"))
    marker = joinpath(gitdir, "spaceagora-companion-source.toml")
    isfile(marker) || throw(ArgumentError("Missing setup ownership record in $path"))
    owned = TOML.parsefile(marker)
    get(owned, "url", nothing) == core.url && get(owned, "revision", nothing) == core.revision ||
        throw(ArgumentError("Managed core source identity does not match the requested pin: $path"))
    readchomp(source_git("-C", path, "remote", "get-url", "origin")) == core.url ||
        throw(ArgumentError("Managed core origin does not match the requested source: $path"))
    head = readchomp(source_git("-C", path, "rev-parse", "HEAD"))
    head == core.revision || throw(ArgumentError("Managed core HEAD $head does not match $(core.revision): $path"))
    clean = readchomp(source_git("-C", path, "status", "--porcelain=v1", "--untracked-files=all", "--ignore-submodules=none"))
    isempty(clean) || throw(ArgumentError("Managed core checkout has local changes; refusing to overwrite them: $path"))
    modules = joinpath(gitdir, "modules")
    !isdir(modules) || isempty(readdir(modules)) ||
        throw(ArgumentError("Managed core checkout must not have initialized submodules: $path"))
    project = TOML.parsefile(joinpath(path, "Project.toml"))
    project["name"] == "SpaceAGORA" && project["uuid"] == "afbfb69f-5c0b-4832-b760-43725dff8540" ||
        throw(ArgumentError("Pinned checkout is not the SpaceAGORA package: $path"))
    return path
end

function acquire_core(plan)
    plan.core.path === nothing || return plan.core.path  # the explicit core checkout is never mutated
    sources = joinpath(plan.destination, "sources")
    islink(sources) && throw(ArgumentError("Refusing a symlinked managed sources directory: $sources"))
    mkpath(sources)
    path = joinpath(sources, "SpaceAGORA-" * plan.core.revision)
    if ispath(path) || islink(path)
        return verify_core_checkout(path, plan.core)
    end
    # Build under a temporary sibling, then publish only a verified checkout.
    # A failed fetch or validation removes only this newly created temporary tree.
    mktempdir(sources; prefix=".core-") do temporary
        run(source_git("init", "--quiet", temporary))
        run(source_git("-C", temporary, "remote", "add", "origin", plan.core.url))
        run(source_git("-C", temporary, "fetch", "--quiet", "--no-tags", "--no-recurse-submodules", "--depth=1", "origin", plan.core.revision))
        run(source_git("-C", temporary, "checkout", "--quiet", "--no-recurse-submodules", "--detach", plan.core.revision))
        open(joinpath(temporary, ".git", "spaceagora-companion-source.toml"), "w") do io
            TOML.print(io, Dict("url" => plan.core.url, "revision" => plan.core.revision))
        end
        verify_core_checkout(temporary, plan.core)
        mv(temporary, path)
    end
    return path
end

function setup(plan; precompile::Bool=true)
    core_path = acquire_core(plan)  # verify source before any Pkg environment mutation
    previous = Base.active_project()
    try
        withenv("JULIA_PKG_PRECOMPILE_AUTO" => "0") do
            Pkg.activate(plan.destination)
            # Explicit core installation is intentional: dependency [sources]
            # propagation is not assumed, including on Julia 1.12.
            Pkg.develop(Pkg.PackageSpec(path=core_path))
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
