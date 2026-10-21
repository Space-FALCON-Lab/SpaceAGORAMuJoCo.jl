# Included by load_order.jl once both packages have loaded, so it runs in this
# repository's CI and in SpaceAGORA's, which tests this package against the core it
# changes. The package reaches SpaceAGORA only through names the
# ExternalPropagationServices contract lists and through SpaceAGORA's public root
# names: a contract change that drops a name the package uses, or a new use of an
# unlisted or private name, fails here.
@testset "MuJoCo uses only listed contract names and public SpaceAGORA names" begin
    services = SpaceAGORA.ExternalPropagationServices
    # Besides the two inventories, every adapter uses the version check.
    listed = Set{Symbol}((services.IMPLEMENTED_FUNCTIONS..., services.CONSUMED_SERVICES..., :CompatibilityError, :require_version))
    contract = Symbol[]
    root = Symbol[]
    for file in readdir(joinpath(@__DIR__, "..", "src"); join=true)
        code = join((first(split(line, '#'; limit=2)) for line in eachline(file)), '\n')
        for spelled in eachmatch(r"(?<![A-Za-z0-9_.])(?:EP|SpaceAGORA\.ExternalPropagationServices)\.([A-Za-z_][A-Za-z0-9_!]*)", code)
            push!(contract, Symbol(spelled.captures[1]))
            @test Symbol(spelled.captures[1]) in listed
        end
        for spelled in eachmatch(r"SpaceAGORA\.(?!ExternalPropagationServices\b)([A-Za-z_][A-Za-z0-9_!]*)(\.[A-Za-z_])?", code)
            name = Symbol(spelled.captures[1])
            push!(root, name)
            # A public root name, used directly rather than as a path into a module.
            @test Base.ispublic(SpaceAGORA, name) && spelled.captures[2] === nothing
        end
    end
    @test length(unique(contract)) >= 12  # refuses a check that silently found nothing
    @test length(unique(root)) >= 5
    # Names imported into the package's modules follow the same rule.
    from_core(value) = try
        Base.moduleroot(value isa Module ? value : parentmodule(value)) === SpaceAGORA
    catch
        false
    end
    for M in (SpaceAGORAMuJoCo, SpaceAGORAMuJoCo.Binding), name in names(M; all=true, imported=true, usings=true)
        name === :SpaceAGORA && continue
        isdefined(M, name) && Base.binding_module(M, name) !== M || continue
        value = getglobal(M, name)
        from_core(value) || continue
        @test (name in listed && getglobal(services, name) === value) ||
            (Base.ispublic(SpaceAGORA, name) && getglobal(SpaceAGORA, name) === value)
    end
end
