# Explicit standalone installation; shared policy is generated from core's
# scripts/companion_support/installation.jl. No core package is needed to plan.
module CompanionInstallation
using Pkg, TOML
include("support/installation.jl")
using .CompanionInstaller: canonical_destination, inside, source_git, verify_core_checkout, acquire_core
const PACKAGE_ROOT = realpath(joinpath(@__DIR__, ".."))
const ENVIRONMENT_MARKER = ".spaceagora-mujoco-environment.toml"

installation_plan(directory::AbstractString; kwargs...) =
    CompanionInstaller.installation_plan(PACKAGE_ROOT, directory;
        pin_file="SpaceAGORASource.toml", marker_name=ENVIRONMENT_MARKER, kwargs...)

setup(plan; precompile::Bool=true) = CompanionInstaller.setup(plan;
    marker_name=ENVIRONMENT_MARKER, package_name="SpaceAGORAMuJoCo", precompile)

function main(args=ARGS)
    length(args) == 1 || throw(ArgumentError("Usage: julia scripts/setup_env.jl <environment-directory>"))
    plan = installation_plan(only(args))
    println("Companion: ", plan.companion)
    println("Core: ", plan.core.path === nothing ? "$(plan.core.url) @ $(plan.core.revision)" : plan.core.path)
    return setup(plan)
end
end
abspath(PROGRAM_FILE) == abspath(@__FILE__) && CompanionInstallation.main()
