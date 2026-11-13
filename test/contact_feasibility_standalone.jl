# Native-only feasibility command: Julia stdlibs and the pinned artifact; no core import.
# julia --startup-file=no test/contact_feasibility_standalone.jl [report.toml]
using TOML, Dates
length(ARGS)<=1 || error("expected at most one report path")
module SpaceAGORAMuJoCo
include(joinpath(@__DIR__, "..", "src", "binding.jl"))
end
include("native_header_tests.jl")
include("contact_feasibility_tests.jl")
if !isempty(ARGS)
    root=dirname(@__DIR__)
    report=copy(ContactFeasibilityTests.RESULTS)
    report["provenance"]=Dict("julia"=>string(VERSION),"platform"=>Sys.MACHINE,
        "mujoco"=>SpaceAGORAMuJoCo.Binding.version(),"utc"=>string(now(UTC)),
        "revision"=>strip(read(`git -C $root rev-parse HEAD`,String)),
        "dirty"=>!isempty(read(`git -C $root status --porcelain`,String)))
    open(only(ARGS),"w") do io
        TOML.print(io,report;sorted=true)
    end
end
