# Explicit standalone installation; shared policy is generated from core's
# scripts/companion_support/installation.jl. No core package is needed to plan.
module CompanionInstallation
using Pkg, TOML
include("support/installation.jl")
using .CompanionInstaller: canonical_destination, inside, source_git, verify_core_checkout, acquire_core
const PACKAGE_ROOT = realpath(joinpath(@__DIR__, ".."))
const ENVIRONMENT_MARKER = ".spaceagora-mujoco-environment.toml"
const PREFETCH_FLAG = "--prefetch-native"

installation_plan(directory::AbstractString; kwargs...) =
    CompanionInstaller.installation_plan(PACKAGE_ROOT, directory;
        pin_file="SpaceAGORASource.toml", marker_name=ENVIRONMENT_MARKER, kwargs...)

setup(plan; precompile::Bool=true) = CompanionInstaller.setup(plan;
    marker_name=ENVIRONMENT_MARKER, package_name="SpaceAGORAMuJoCo", precompile)

# Name of this host's table in NativeLibrary.toml (the package's Binding.platform_key uses the same rule).
native_platform_key() = string(Sys.isapple() ? "macos" : Sys.islinux() ? "linux" : lowercase(string(Sys.KERNEL)), "_", Sys.ARCH)

"""
What setup does about the native library on platform `key`: `:unsupported` when `NativeLibrary.toml` has
no table for it, `:prefetch` when the library is installed now, `:lazy` when the first native use installs it.
A platform whose source is a disk image is prefetched by default, because that build is slow; a platform
with a Pkg-downloadable artifact stays lazy unless `--prefetch-native` is passed.
"""
function native_action(key::AbstractString=native_platform_key(); requested::Bool=false)
    platforms = get(TOML.parsefile(joinpath(PACKAGE_ROOT, "NativeLibrary.toml")), "platforms", Dict{String,Any}())
    haskey(platforms, key) || return :unsupported
    return requested || platforms[key]["source"] == "dmg" ? :prefetch : :lazy
end

function prefetch_native(environment::AbstractString)
    code = "using SpaceAGORAMuJoCo; println(\"Native library: MuJoCo \", SpaceAGORAMuJoCo.Binding.ensure_native!(), \" is installed and loads\")"
    command = `$(Base.julia_cmd()) --startup-file=no --project=$environment -e $code`
    success(pipeline(addenv(command, "JULIA_LOAD_PATH" => "@:@stdlib"); stdout, stderr)) ||
        error("The Julia packages are installed, but the native MuJoCo library is not: see the error above. " *
            "Run this command again to retry.")
    return nothing
end

function main(args=ARGS)
    directories = filter(!startswith("--"), args)
    flags = filter(startswith("--"), args)
    length(directories) == 1 && all(==(PREFETCH_FLAG), flags) ||
        throw(ArgumentError("Usage: julia scripts/setup_env.jl <environment-directory> [$PREFETCH_FLAG]"))
    plan = installation_plan(only(directories))
    println("Companion: ", plan.companion)
    println("Core: ", plan.core.path === nothing ? "$(plan.core.url) @ $(plan.core.revision)" : plan.core.path)
    environment = setup(plan)
    action = native_action(; requested=PREFETCH_FLAG in flags)
    if action === :prefetch
        prefetch_native(environment)
    elseif action === :unsupported
        println("Native scenes are unavailable on this platform ($(native_platform_key())): ",
            "NativeLibrary.toml lists no MuJoCo build for it. The packages are installed.")
    end
    return environment
end
end
abspath(PROGRAM_FILE) == abspath(@__FILE__) && CompanionInstallation.main()
