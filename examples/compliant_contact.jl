# Run in an environment containing the exact paired core/companion revisions.
using SpaceAGORA, SpaceAGORAMuJoCo, TOML
include("bus_bumper_fixture.jl")
include("contact_measurements.jl")
isdefined(SpaceAGORA,:ContactLoadServices) || error("This example needs the experimental paired core revision; see docs/compliant_contact.md.")
C=SpaceAGORA.ContactLoadServices
reports=Dict{String,Any}()
for power in 10:13
    h=2.0^-power
    args=C.with_contact_model(bus_bumper_configuration(h=h),BusBumperContact())
    sol=run_simulation(args;return_solution=true,visualization=false)
    report=headon_measurements(sol)
    report["core_maximum_step_s"]=h
    report["native_timestep_s"]=2.0^-12
    report["native_version"]=SpaceAGORAMuJoCo.Binding.version()
    report["julia_version"]=string(VERSION)
    report["platform"]=string(Sys.MACHINE)
    report["duration_s"]=last(sol.t)
    reports["step_2_neg_$power"]=report
end
path=isempty(ARGS) ? "contact-engine-report.toml" : only(ARGS)
open(io->TOML.print(io,reports),path,"w")
println("Wrote ",abspath(path))
