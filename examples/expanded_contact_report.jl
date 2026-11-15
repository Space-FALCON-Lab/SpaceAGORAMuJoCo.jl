using TOML, SHA

function expanded_contact_case(name;duration=.5,h=2.0^-12,capture_speed=1e-7,tolerance=1e-10,origin=SVector(0.,0.,0.))
    if startswith(name,"arm_")
        case=arm_contact_configuration(;duration,h,capture_speed,tolerance,origin,
            friction=name=="arm_frictionless" ? 0. : .4,
            simultaneous=name=="arm_simultaneous",exact_sticking=name=="arm_exact")
        return (;case...,arm=true)
    end
    geometry=(ContactGeometry(1,:box,(.5,.4,.3)),ContactGeometry(2,:sphere,(.1,)))
    args=bus_bumper_configuration(;duration,h,tolerance,origin,gap=-.001,normal_speed=0.,rest_gap=-.01,
        tangent=name=="exact_hold" ? 0. : .02,rest_tangent=name=="exact_reversal" ? .02 : .001)
    (;args,contact=CoulombContact(geometry,((1,2),);capture_speed),arm=false)
end
expanded_ledger(case,sol;kwargs...)=case.arm ? arm_contact_ledger(case,sol;kwargs...) : coulomb_trajectory_ledger(case.args,sol,case.contact;kwargs...)
function expanded_states(sol,t)
    sc=sol(t;continuity=:right).sc[1];origin=first(sol.u).sc[1].pos
    states=[(;r=SVector{3,Float64}(sc.pos-origin),v=SVector{3,Float64}(sc.vel),q=normalize(SVector{4,Float64}(sc.q)),w=SVector{3,Float64}(sc.ω))]
    arm=hasproperty(sc,:arm_r)&&size(sc.arm_r,2)>0
    for i in 1:(arm ? size(sc.arm_r,2) : size(sc.att_r,2))
        push!(states,arm ? (;r=SVector{3,Float64}(sc.arm_r[:,i]-origin),v=SVector{3,Float64}(sc.arm_v[:,i]),q=normalize(SVector{4,Float64}(sc.arm_q[:,i])),w=SVector{3,Float64}(sc.arm_ω[:,i])) :
            (;r=states[1].r+SVector{3,Float64}(sc.att_r[:,i]),v=states[1].v+SVector{3,Float64}(sc.att_v[:,i]),q=normalize(SVector{4,Float64}(sc.att_q[:,i])),w=SVector{3,Float64}(sc.att_ω[:,i])))
    end
    states
end
function expanded_trajectory_difference(a,b)
    maximum(max(norm(x.r-y.r),norm(x.v-y.v)/.25,norm(x.w-y.w)/.25,contact_angle(x.q,y.q))
        for t in range(0.,last(a.t);length=1025) for (x,y) in zip(expanded_states(a,t),expanded_states(b,t)))
end
function expanded_gates(name,ledger)
    m=ledger.metrics
    gates=Dict("energy_work"=>m["energy_work_residual_J"]<1.3125e-5,
        "linear_momentum"=>m["momentum_residual_kg_m_s"]<5.25e-7,
        "angular_momentum"=>m["angular_momentum_residual_kg_m2_s"]<5.25e-7,
        "passive_friction"=>m["maximum_friction_power_W"]<=1e-10&&m["friction_work_J"]<=1e-10,
        "friction_cone"=>m["maximum_cone_ratio"]<=1+1e-10,
        "projection_energy"=>-1e-10<=m["projection_work_J"]<=1e-12,
        "force_balance"=>m["force_balance_residual_N"]<1e-10,
        "moment_balance"=>m["moment_balance_residual_Nm"]<1e-10,
        "power_projection"=>m["power_projection_residual_W"]<1e-10)
    if startswith(name,"exact_")||name=="arm_exact"
        gates["zero_slip_hold"]=m["maximum_sticking_speed_m_s"]<1e-10&&any(x->x["sticking"],m["transitions"])
        haskey(m,"sticking_slip_distance_m") && (gates["zero_slip_displacement"]=m["sticking_slip_distance_m"]<1e-9)
    end
    if name=="exact_reversal"
        gates["release_and_resticking"]=count(x->!x["sticking"],m["transitions"])>=2&&count(x->x["sticking"],m["transitions"])>=3
    elseif startswith(name,"arm_")
        gates["onset"]=m["initial_gap_m"]>0&&m["minimum_gap_m"]<0
        gates["joint_rotation"]=m["maximum_relative_joint_angle_rad"]<pi/4
        if name=="arm_simultaneous"
            gates["simultaneous_contacts"]=m["maximum_contacts"]>=2
        else
            first_contact=findfirst(r->r.gap<0,ledger.rows)
            gates["separation"]=first_contact!==nothing&&any(r->r.gap>0,ledger.rows[first_contact:end])
        end
    end
    gates
end
function write_expanded_trajectory(path,sol,ledger)
    times=collect(range(0.,last(sol.t);length=ceil(Int,last(sol.t)/2.0^-11)+1))
    count=length(expanded_states(sol,0.))
    open(path,"w") do io
        headers=["time_s"]
        for i in 1:count, (kind,axes) in (("r",("x","y","z")),("v",("x","y","z")),("q",("x","y","z","w")),("w",("x","y","z")))
            append!(headers,["body$(i)_$(kind)_$axis" for axis in axes])
        end
        println(io,join(headers,','))
        for t in times
            values=[t]
            for s in expanded_states(sol,t)
                append!(values,vcat(s.r,s.v,s.q,s.w))
            end
            println(io,join(values,','))
        end
    end
end
function expanded_source(root)
    command(args...)=readchomp(Cmd(["git","-C",root,args...]))
    Dict("revision"=>command("rev-parse","HEAD"),"dirty"=>!isempty(command("status","--porcelain","--untracked-files=normal")))
end
function write_expanded_contact_report(directory;duration=.5)
    mkpath(directory)
    root=dirname(@__DIR__);core=dirname(dirname(pathof(SpaceAGORA)))
    files=String[]
    for parent in ("src","test","examples"), (dir,_,names) in walkdir(joinpath(root,parent)), name in names
        endswith(name,".jl")&&push!(files,relpath(joinpath(dir,name),root))
    end
    report=Dict{String,Any}("schema_version"=>1,"core"=>expanded_source(core),"companion"=>expanded_source(root),
        "source_sha256"=>Dict(file=>bytes2hex(sha256(read(joinpath(root,file)))) for file in files),
        "platform"=>string(Sys.MACHINE),"julia_version"=>string(VERSION),"native_version"=>SpaceAGORAMuJoCo.Binding.version(),
        "contract_version"=>string(SpaceAGORA.ContactLoadServices.CONTRACT_VERSION),"duration_s"=>duration,
        "solver"=>Dict("algorithm"=>"Tsit5","relative_tolerance"=>1e-10,"absolute_tolerance"=>1e-11,
            "capture_speed_m_s"=>1e-7,"force_residual_limit_N"=>1e-10),
        "scales"=>Dict("length_m"=>1.,"velocity_m_s"=>.25,"momentum_kg_m_s"=>5.25,"energy_J"=>1.3125),
        "cases"=>Dict{String,Any}(),"qualification_failures"=>String[])
    for name in ("exact_hold","exact_arrest","exact_reversal","arm_frictionless","arm_regularized","arm_exact","arm_simultaneous")
        println("Qualifying ",name," for ",duration," seconds");flush(stdout)
        solutions=[];ledgers=[];entries=Dict{String,Any}[]
        for power in 10:13
            println("  maximum step 2^-",power);flush(stdout)
            case=expanded_contact_case(name;duration,h=2.0^-power)
            sol=run_simulation(SpaceAGORA.ContactLoadServices.with_contact_model(case.args,case.contact);return_solution=true,visualization=false)
            last(sol.t)==duration || error("Incomplete $name integration")
            ledger=expanded_ledger(case,sol);gates=expanded_gates(name,ledger)
            push!(solutions,sol);push!(ledgers,ledger)
            push!(entries,merge(copy(ledger.metrics),Dict("core_maximum_step_s"=>2.0^-power,"gates"=>gates)))
            append!(report["qualification_failures"],["$name:step_$power:$gate" for (gate,ok) in gates if !ok])
        end
        impulse_key=startswith(name,"arm_") ? "bus_contact_impulse_N_s" : "bumper_contact_impulse_N_s"
        for (entry,sol) in zip(entries,solutions)
            entry["scaled_trajectory_error"]=expanded_trajectory_difference(sol,last(solutions))
            entry["scaled_impulse_error"]=norm(entry[impulse_key]-last(entries)[impulse_key])/5.25
        end
        gates=Dict("trajectory_refinement"=>entries[end-1]["scaled_trajectory_error"]<1e-3,
            "impulse_refinement"=>entries[end-1]["scaled_impulse_error"]<1e-3)
        finecase=expanded_contact_case(name;duration,h=2.0^-13)
        finequad=expanded_ledger(finecase,last(solutions);sample_step=2.0^-12)
        quadrature_difference=abs(finequad.metrics["contact_work_J"]-last(entries)["contact_work_J"])
        gates["work_quadrature"]=quadrature_difference<1.3125e-6
        sensitivity=Dict{String,Any}[]
        if startswith(name,"exact_")||name=="arm_exact"
            for capture in (1e-6,1e-8)
                println("  capture tolerance ",capture);flush(stdout)
                case=expanded_contact_case(name;duration,h=2.0^-13,capture_speed=capture)
                sol=run_simulation(SpaceAGORA.ContactLoadServices.with_contact_model(case.args,case.contact);return_solution=true,visualization=false)
                ledger=expanded_ledger(case,sol);checks=expanded_gates(name,ledger)
                difference=expanded_trajectory_difference(sol,last(solutions))
                checks["capture_refinement"]=difference<1e-3
                push!(sensitivity,merge(ledger.metrics,Dict("capture_speed_m_s"=>capture,"scaled_trajectory_error"=>difference,"gates"=>checks)))
                append!(report["qualification_failures"],["$name:capture_$capture:$gate" for (gate,ok) in checks if !ok])
            end
        end
        numerical_checks=Dict{String,Any}[]
        if name in ("exact_reversal","arm_exact")
            for tolerance in (1e-9,1e-11)
                println("  integration tolerance ",tolerance);flush(stdout)
                case=expanded_contact_case(name;duration,h=2.0^-13,tolerance)
                sol=run_simulation(SpaceAGORA.ContactLoadServices.with_contact_model(case.args,case.contact);return_solution=true,visualization=false)
                ledger=expanded_ledger(case,sol);checks=expanded_gates(name,ledger)
                difference=expanded_trajectory_difference(sol,last(solutions))
                checks["integration_tolerance_refinement"]=difference<1e-3
                push!(numerical_checks,merge(ledger.metrics,Dict("relative_tolerance"=>tolerance,"scaled_trajectory_error"=>difference,"gates"=>checks)))
                append!(report["qualification_failures"],["$name:tolerance_$tolerance:$gate" for (gate,ok) in checks if !ok])
            end
            case=expanded_contact_case(name;duration,h=2.0^-13,origin=SVector(7e6,0.,0.))
            println("  shifted orbital origin");flush(stdout)
            sol=run_simulation(SpaceAGORA.ContactLoadServices.with_contact_model(case.args,case.contact);return_solution=true,visualization=false)
            ledger=expanded_ledger(case,sol);checks=expanded_gates(name,ledger)
            difference=expanded_trajectory_difference(sol,last(solutions))
            checks["translated_origin_invariance"]=difference<1e-3
            push!(numerical_checks,merge(ledger.metrics,Dict("origin_x_m"=>7e6,"scaled_trajectory_error"=>difference,"gates"=>checks)))
            append!(report["qualification_failures"],["$name:translated_origin:$gate" for (gate,ok) in checks if !ok])
        end
        append!(report["qualification_failures"],["$name:$gate" for (gate,ok) in gates if !ok])
        file=name*".csv";write_expanded_trajectory(joinpath(directory,file),last(solutions),last(ledgers))
        report["cases"][name]=Dict("refinements"=>entries,"capture_refinements"=>sensitivity,"numerical_checks"=>numerical_checks,"gates"=>gates,
            "quadrature_work_difference_J"=>quadrature_difference,"trajectory_file"=>file)
        # Preserve completed evidence even if a later fixture fails explicitly.
        report["qualified"]=false
        open(io->TOML.print(io,report;sorted=true),joinpath(directory,"summary.toml"),"w")
    end
    report["qualified"]=isempty(report["qualification_failures"])
    open(io->TOML.print(io,report;sorted=true),joinpath(directory,"summary.toml"),"w")
    report
end
