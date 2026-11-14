# Reports every gate without claiming that a passing bookkeeping ledger qualifies
# friction. --require-qualified is a strict gate and currently exits nonzero.
using SpaceAGORA, SpaceAGORAMuJoCo
include("bus_bumper_fixture.jl")
include("contact_validation.jl")
include("contact_validation_report.jl")
paths=filter(!=("--require-qualified"),ARGS)
length(paths)<=1 || error("Usage: validate_contact.jl [output-directory] [--require-qualified]")
report=write_contact_validation_report(isempty(paths) ? "contact-validation" : only(paths))
println("Physical qualification: ",report["qualified"] ? "passed" : "BLOCKED")
foreach(println,report["qualification_failures"])
"--require-qualified" in ARGS && !report["qualified"] && exit(1)
