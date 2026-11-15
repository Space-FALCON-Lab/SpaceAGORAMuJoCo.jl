module ExpandedComparisonTests
using Test, TOML
include("../examples/compare_expanded_contact_platforms.jl")
@testset "expanded platform comparison rejects incompatible or failed evidence" begin
    mktempdir() do dir
        a=joinpath(dir,"left");b=joinpath(dir,"right");mkpath(a);mkpath(b)
        state=[0.,0.,0.,0.,0.,0.,0.,0.,0.,1.,0.,0.,0.]
        header="time_s,"*join(["body1_$(field)_$axis" for (field,axes) in (("r",("x","y","z")),("v",("x","y","z")),("q",("x","y","z","w")),("w",("x","y","z"))) for axis in axes],',')
        trajectory=header*"\n"*join([0.;state],',')*"\n"*join([1.;state],',')*"\n"
        case=Dict("trajectory_file"=>"case.csv","refinements"=>[Dict("core_maximum_step_s"=>2.0^-13,
            "bumper_contact_impulse_N_s"=>[0.,0.,0.],"contact_work_J"=>0.,"transitions"=>[])])
        source=Dict("revision"=>"synthetic-test-fixture","dirty"=>false)
        report=Dict("schema_version"=>1,"core"=>source,"companion"=>source,"source_sha256"=>Dict(),
            "native_version"=>3011000,"contract_version"=>"0.2.0","duration_s"=>1.,"scales"=>Dict(),"solver"=>Dict(),
            "platform"=>"synthetic","qualified"=>true,"cases"=>Dict("exact_hold"=>case))
        save(path,r)=open(io->TOML.print(io,r),joinpath(path,"summary.toml"),"w")
        for path in (a,b)
            write(joinpath(path,"case.csv"),trajectory);save(path,report)
        end
        @test compare_expanded_contact_platforms(a,b)["cross_platform_agreement"]
        bad=deepcopy(report);bad["qualified"]=false;save(b,bad)
        @test !compare_expanded_contact_platforms(a,b)["physically_qualified"]
        bad=deepcopy(report);bad["contract_version"]="wrong";save(b,bad)
        @test_throws ErrorException compare_expanded_contact_platforms(a,b)
        bad=deepcopy(report);bad["core"]["dirty"]=true;save(a,bad);save(b,bad)
        @test_throws ErrorException compare_expanded_contact_platforms(a,b)
        save(a,report);save(b,report)
        altered=copy(state);altered[4]=.01
        write(joinpath(b,"case.csv"),header*"\n"*join([0.;state],',')*"\n"*join([1.;altered],',')*"\n")
        @test !compare_expanded_contact_platforms(a,b)["cross_platform_agreement"]
        write(joinpath(b,"case.csv"),replace(trajectory,"1.0"=>"NaN"))
        @test_throws ErrorException compare_expanded_contact_platforms(a,b)
    end
end
end
