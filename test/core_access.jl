# Checks how a companion's runtime code reaches SpaceAGORA. The same file serves
# the service-boundary tests of SpaceAGORAEDG, HYPR and SpaceAGORAMuJoCo; each
# states its own policy: the names it may take from a SpaceAGORA module
# (`allowed_member(module, name)`) and the SpaceAGORA modules it may step through.
#
# - `runtime_violations`: bindings of the companion's modules whose value comes
#   from SpaceAGORA and is not permitted, whether imported or defined there (a
#   local alias such as `const T = SomeAlias.SimulationModel.ODEParams` too).
# - `source_violations`: in module code and function bodies, every access that
#   starts at SpaceAGORA or at a local alias of a SpaceAGORA module (`X = ...`,
#   `X = getfield(...)`, `import ... as X`) and steps through a name that is not
#   permitted; `getfield`/`getproperty`/`getglobal` on such a module with a
#   computed name; and the SpaceAGORA module passed on as a value. Strings and
#   comments are not code, so documentation is not checked.
module CoreAccess
export CorePolicy, runtime_violations, source_violations

struct CorePolicy
    core::Module
    allowed_member::Function
    modules::Vector{Module}
    values::Vector{Any}
end

function CorePolicy(core::Module, allowed_member::Function, modules)
    values = Any[]
    for m in modules, name in names(m; all=true, imported=true, usings=true)
        isdefined(m, name) && allowed_member(m, name) || continue
        value = getglobal(m, name)
        value isa Module || push!(values, value)
    end
    return CorePolicy(core, allowed_member, collect(Module, modules), values)
end

owner(value) = try
    value isa Module ? value : value isa Union{Function, Type} ? parentmodule(value) : parentmodule(typeof(value))
catch
    nothing
end
from_core(policy, value) = (o = owner(value); o !== nothing && Base.moduleroot(o) === policy.core)
allowed_module(policy, m) = any(x -> x === m, policy.modules)
permitted(policy, value) = value isa Module ? allowed_module(policy, value) :
    value isa Union{Function, Type} ? any(v -> v === value, policy.values) :
    any(v -> v isa Type && typeof(value) <: v, policy.values)

"""Bindings of `root` and its submodules whose SpaceAGORA value is not permitted."""
function runtime_violations(policy::CorePolicy, root::Module)
    found = String[]
    modules = Module[root]
    for M in modules, name in names(M; all=true, imported=true, usings=true)
        isdefined(M, name) || continue
        value = getglobal(M, name)
        if value isa Module && value !== M && parentmodule(value) === M && Base.moduleroot(value) === Base.moduleroot(root)
            push!(modules, value)
        elseif from_core(policy, value) && !permitted(policy, value)
            push!(found, "$M.$name")
        end
    end
    return found
end

"""Source accesses in `files` that are not permitted, and the permitted `(module, name)` steps seen."""
function source_violations(policy::CorePolicy, files)
    core = policy.core
    exprs = [Meta.parseall(read(file, String); filename=file) for file in files]
    aliases = Dict{Symbol, Module}(nameof(core) => core)
    found = String[]
    used = Set{Tuple{Module, Symbol}}()
    function step(value, name, record)
        value isa Module && Base.moduleroot(value) === core || return nothing
        if !(allowed_module(policy, value) && policy.allowed_member(value, name) && isdefined(value, name))
            record && push!(found, "$(value).$(name)")
            return nothing
        end
        record && push!(used, (value, name))
        return Some(getglobal(value, name))
    end
    is_access(ex) = Meta.isexpr(ex, :.) && length(ex.args) == 2 && ex.args[2] isa QuoteNode
    is_lookup(ex) = Meta.isexpr(ex, :call) && length(ex.args) >= 3 &&
        ex.args[1] in (:getfield, :getproperty, :getglobal, :isdefined)
    function resolve(ex, record)
        if ex isa Symbol
            return haskey(aliases, ex) ? Some(aliases[ex]) : nothing
        elseif is_access(ex)
            base = resolve(ex.args[1], record)
            return base === nothing ? nothing : step(something(base), ex.args[2].value, record)
        elseif is_lookup(ex) && ex.args[1] !== :isdefined
            base = resolve(ex.args[2], record)
            base === nothing && return nothing
            key = ex.args[3]
            if !(key isa QuoteNode && key.value isa Symbol)
                record && push!(found, "computed name: $(ex)")
                return nothing
            end
            return step(something(base), key.value, record)
        end
        return nothing
    end
    import_path(ex) = Meta.isexpr(ex, :.) && all(a -> a isa Symbol, ex.args) ? ex.args : nothing
    function resolve_path(path, record)
        (isempty(path) || !haskey(aliases, path[1])) && return nothing
        value = Some(aliases[path[1]])
        for name in path[2:end]
            value = step(something(value), name, record)
            value === nothing && return nothing
        end
        return value
    end
    # Aliases, wherever they are defined, until no new one appears.
    function collect!(ex)
        ex isa Expr || return false
        changed = false
        if Meta.isexpr(ex, :(=)) && ex.args[1] isa Symbol && !haskey(aliases, ex.args[1])
            value = resolve(ex.args[2], false)
            if value !== nothing && something(value) isa Module
                aliases[ex.args[1]] = something(value)
                changed = true
            end
        elseif ex.head in (:import, :using)
            for arg in ex.args
                Meta.isexpr(arg, :as) && arg.args[2] isa Symbol && !haskey(aliases, arg.args[2]) || continue
                path = import_path(arg.args[1])
                value = path === nothing ? nothing : resolve_path(path, false)
                if value !== nothing && something(value) isa Module
                    aliases[arg.args[2]] = something(value)
                    changed = true
                end
            end
        end
        for arg in ex.args
            changed |= collect!(arg)
        end
        return changed
    end
    while any(collect!, exprs)
    end
    base_of(ex) = is_access(ex) ? base_of(ex.args[1]) : ex
    function check!(ex)
        if ex isa Symbol
            get(aliases, ex, nothing) === core && push!(found, "the $(nameof(core)) module used as a value ($ex)")
        elseif !(ex isa Expr)
            return
        elseif is_access(ex)
            base = base_of(ex)
            base isa Symbol && haskey(aliases, base) ? resolve(ex, true) : check!(base)
        elseif is_lookup(ex) && resolve(ex.args[2], false) !== nothing
            ex.args[1] === :isdefined || resolve(ex, true)
            foreach(check!, ex.args[4:end])
        elseif Meta.isexpr(ex, :(=)) && ex.args[1] isa Symbol && resolve(ex.args[2], false) !== nothing
            resolve(ex.args[2], true)
        elseif ex.head in (:import, :using)
            for arg in ex.args
                target = Meta.isexpr(arg, :as) ? arg.args[1] : arg
                if Meta.isexpr(target, :(:))
                    path = import_path(target.args[1])
                    value = path === nothing ? nothing : resolve_path(path, true)
                    value === nothing && continue
                    for item in target.args[2:end]
                        name = import_path(Meta.isexpr(item, :as) ? item.args[1] : item)
                        name === nothing || step(something(value), only(name), true)
                    end
                else
                    path = import_path(target)
                    path === nothing || resolve_path(path, true)
                end
            end
        else
            foreach(check!, ex.args)
        end
        return nothing
    end
    foreach(check!, exprs)
    return found, used
end
end # module CoreAccess
