# Included by load_order.jl once both packages have loaded, so it runs in this
# repository's CI and in SpaceAGORA's, which tests this package against the core it
# changes. The package reaches SpaceAGORA only through names the
# ExternalPropagationServices contract lists and through SpaceAGORA's public root
# names: a contract change that drops a name the package uses, or a new use of an
# unlisted or private name, fails here. The check covers the runtime source under
# src/, in module code and function bodies, through the bindings of SpaceAGORA
# modules that test/core_access.jl follows and within the limits it states; the
# tests' private fixtures are outside it.
include("core_access.jl")
using .CoreAccess

@testset "MuJoCo uses only listed contract names and public SpaceAGORA names" begin
    services = SpaceAGORA.ExternalPropagationServices
    # From SpaceAGORA the facade and public root names other than modules; from the
    # facade its two inventories and the version check every adapter makes.
    listed = Set{Symbol}((services.IMPLEMENTED_FUNCTIONS..., services.CONSUMED_SERVICES..., :CompatibilityError, :require_version))
    function allowed(m, name)
        m === services && return name in listed
        m === SpaceAGORA || return false
        name === :ExternalPropagationServices && return true
        return Base.ispublic(SpaceAGORA, name) && isdefined(SpaceAGORA, name) && !(getglobal(SpaceAGORA, name) isa Module)
    end
    policy = CorePolicy(SpaceAGORA, allowed, (SpaceAGORA, services))

    @test isempty(runtime_violations(policy, SpaceAGORAMuJoCo))
    sources = [joinpath(directory, file) for (directory, _, files) in walkdir(joinpath(@__DIR__, "..", "src"))
               for file in files if endswith(file, ".jl")]
    found, used, known = source_violations(policy, sources)
    @test isempty(found)
    # Every binding of a SpaceAGORA module is one the source check follows.
    @test isempty(unknown_module_bindings(policy, SpaceAGORAMuJoCo, known))
    # A check that saw nothing would pass too: the source does name the contract's
    # services and public root names.
    @test length(sources) >= 4
    @test count(((m, _),) -> m === services, used) >= 12
    @test count(((m, name),) -> m === SpaceAGORA && name !== :ExternalPropagationServices, used) >= 5
end
