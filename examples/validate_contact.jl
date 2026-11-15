# Reports every gate without claiming that a passing bookkeeping ledger qualifies
# friction. --require-qualified makes every physical gate mandatory.
using SpaceAGORA, SpaceAGORAMuJoCo
include("bus_bumper_fixture.jl")
include("contact_validation.jl")
include("contact_validation_report.jl")
paths=filter(!=("--require-qualified"),ARGS)
length(paths)<=1 || error("Usage: validate_contact.jl [output-directory] [--require-qualified]")
duration=parse(Float64,get(ENV,"CONTACT_VALIDATION_DURATION_S","0.25"))
isfinite(duration) && duration>0 || error("CONTACT_VALIDATION_DURATION_S must be positive and finite")
report=write_contact_validation_report(isempty(paths) ? "contact-validation" : only(paths);duration=duration)
println("Physical qualification: ",report["qualified"] ? "passed" : "BLOCKED")
foreach(println,report["qualification_failures"])
"--require-qualified" in ARGS && !report["qualified"] && exit(1)
