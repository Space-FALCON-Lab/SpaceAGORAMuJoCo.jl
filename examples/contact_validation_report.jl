# Included after bus_bumper_fixture.jl and contact_validation.jl.
using TOML, SHA

function contact_source_record(root)
    command(args...) = try
        readchomp(Cmd(["git","-C",root,args...]))
    catch
        "unavailable"
    end
    return Dict("revision"=>command("rev-parse","HEAD"),
        "dirty"=>!isempty(command("status","--porcelain","--untracked-files=normal")))
end

function write_contact_validation_report(directory; duration=.25)
    mkpath(directory)
    start=time_ns()
    C=SpaceAGORA.ContactLoadServices
    companion=dirname(@__DIR__); core=dirname(dirname(pathof(SpaceAGORA)))
    hashes=Dict(file=>bytes2hex(sha256(read(joinpath(companion,file)))) for file in
        ("examples/bus_bumper_fixture.jl","examples/contact_validation.jl",
         "examples/contact_validation_report.jl","test/contact_validation_tests.jl",
         "src/compliant_contact.jl","Artifacts.toml"))
    report=Dict{String,Any}("schema_version"=>1,"core"=>contact_source_record(core),
        "companion"=>contact_source_record(companion),"source_sha256"=>hashes,
        "julia_version"=>string(VERSION),"platform"=>string(Sys.MACHINE),
        "native_version"=>SpaceAGORAMuJoCo.Binding.version(),
        "contact_contract_version"=>"0.1.0",
        "frame"=>"fixed initial bus origin translated at constant initial system COM velocity; axes parallel to inertial axes",
        "scales"=>Dict("mass_kg"=>21.,"length_m"=>1.,"velocity_m_s"=>.25,
            "momentum_kg_m_s"=>5.25,"angular_momentum_kg_m2_s"=>5.25,"energy_J"=>1.3125),
        "core_solver"=>Dict("algorithm"=>"Tsit5","relative_tolerance"=>1e-10,"absolute_tolerance"=>1e-11),
        "native_solver"=>Dict("solver"=>"Newton","cone"=>"elliptic","iterations"=>100,
            "tolerance"=>1e-12,"residual_limit_N"=>1e-10,"timestep_s"=>2.0^-12,
            "time_constant_s"=>.01,"damping_ratio"=>1.,"impedance"=>.9,
            "warm_start"=>false,"sleep"=>false,"diagonal_exact"=>true),
        "cases"=>Dict{String,Any}(),"qualification_failures"=>String[])
    for case in contact_validation_cases()
        contact=SpaceAGORAMuJoCo.BusBumperContact(friction=case.friction)
        entries=Dict{String,Any}[]; sols=[]; ledgers=[]
        for power in 10:13
            args=bus_bumper_configuration(duration=duration,h=2.0^-power,
                offset=case.offset,tangent=case.tangent,translation_damping=case.damping)
            try
                sol=run_simulation(C.with_contact_model(args,contact);return_solution=true,visualization=false)
                ledger=contact_trajectory_ledger(args,sol,contact)
                push!(sols,sol); push!(ledgers,ledger)
                push!(entries,merge(copy(ledger.metrics),Dict("core_maximum_step_s"=>2.0^-power,"status"=>"completed")))
            catch err
                message=sprint(showerror,err)
                startswith(message,"Unconverged contact solve:") || rethrow()
                push!(entries,Dict("core_maximum_step_s"=>2.0^-power,"status"=>"failed","error"=>message))
                push!(report["qualification_failures"],case.name*":step_2_neg_$power")
            end
        end
        if isempty(ledgers)
            report["cases"][case.name]=Dict("refinements"=>entries,"all_runs_failed"=>true)
            continue
        end
        completed=filter(e->e["status"]=="completed",entries)
        fine=last(ledgers); metrics=last(completed)
        args=bus_bumper_configuration(duration=duration,h=2.0^-13,
            offset=case.offset,tangent=case.tangent,translation_damping=case.damping)
        quadrature=contact_trajectory_ledger(args,last(sols),contact;sample_step=2.0^-12)
        metrics["quadrature_work_difference_J"]=abs(quadrature.metrics["contact_work_J"]-metrics["contact_work_J"])
        for (entry,sol,ledger) in zip(completed,sols,ledgers)
            entry["scaled_trajectory_error"]=contact_trajectory_difference(sol,last(sols))
            entry["scaled_impulse_error"]=norm(ledger.metrics["bumper_contact_impulse_N_s"]-fine.metrics["bumper_contact_impulse_N_s"])/5.25
        end
        passed=Dict(
            "all_refinements_completed"=>length(completed)==4,
            "energy_work"=>metrics["energy_work_residual_J"]/1.3125<1e-5,
            "linear_momentum"=>metrics["momentum_residual_kg_m_s"]/5.25<1e-7,
            "angular_momentum"=>metrics["angular_momentum_residual_kg_m2_s"]/5.25<1e-7,
            "work_quadrature"=>metrics["quadrature_work_difference_J"]/1.3125<1e-6,
            "trajectory_refinement"=>length(completed)>=2 && completed[end-1]["scaled_trajectory_error"]<1e-3,
            "impulse_refinement"=>length(completed)>=2 && completed[end-1]["scaled_impulse_error"]<1e-3,
            "separated_endpoints"=>first(fine.rows).gap>0 && last(fine.rows).gap>0,
            "endpoint_dissipation"=>metrics["mechanical_energy_change_J"]<=1e-10,
            "small_joint_rotation"=>metrics["maximum_relative_joint_angle_rad"]<pi/4)
        if case.name=="sliding"
            passed["opposing_sliding_power"]=metrics["sliding_power_qualified"]
        end
        # This extra diagnostic is stricter than the plan's sliding criterion;
        # retain it explicitly rather than treating net friction work as hidden loss.
        if case.name=="low_slip"
            passed["nonpositive_net_friction_work"]=metrics["friction_work_J"]<=1e-10
        end
        append!(report["qualification_failures"],[case.name*":"*name for (name,value) in passed if !value])
        report["cases"][case.name]=Dict("mount_offset_m"=>case.offset,
            "initial_tangential_velocity_m_s"=>case.tangent,"friction_coefficient"=>case.friction,
            "joint_translation_damping_N_s_m"=>case.damping,"refinements"=>entries,
            "gates"=>passed,"trajectory_file"=>case.name*".csv")
        write_contact_trajectory(joinpath(directory,case.name*".csv"),fine)
    end
    # Physical-law sensitivity is separate from numerical convergence: changing
    # solref changes the model, so no equality across these runs is asserted.
    regularization=Dict{String,Any}[]
    for tau in (.005,.01,.02)
        contact=SpaceAGORAMuJoCo.BusBumperContact(time_constant=tau)
        args=bus_bumper_configuration(duration=duration,h=2.0^-13)
        sol=run_simulation(C.with_contact_model(args,contact);return_solution=true,visualization=false)
        entry=contact_trajectory_ledger(args,sol,contact).metrics
        entry["time_constant_s"]=tau
        push!(regularization,entry)
    end
    report["physical_regularization_sweep"]=regularization
    recoveries=Dict{String,Any}[]
    for tol in (1e-13,1e-14)
        contact=SpaceAGORAMuJoCo.BusBumperContact(friction=.4,tolerance=tol)
        args=bus_bumper_configuration(duration=duration,h=2.0^-11,offset=.2,tangent=.15,translation_damping=.2)
        sol=run_simulation(C.with_contact_model(args,contact);return_solution=true,visualization=false)
        entry=contact_trajectory_ledger(args,sol,contact).metrics
        entry["native_tolerance"]=tol
        entry["core_maximum_step_s"]=2.0^-11
        push!(recoveries,entry)
    end
    report["damped_solver_recovery"]=recoveries
    open(joinpath(directory,"convergence.csv"),"w") do io
        println(io,"case,maximum_step_s,scaled_trajectory_error,scaled_impulse_error,status")
        for name in sort!(collect(keys(report["cases"])))
            for entry in report["cases"][name]["refinements"]
                println(io,join((name,entry["core_maximum_step_s"],get(entry,"scaled_trajectory_error",NaN),get(entry,"scaled_impulse_error",NaN),entry["status"]),','))
            end
        end
    end
    report["qualified"]=isempty(report["qualification_failures"])
    report["elapsed_seconds"]=(time_ns()-start)/1e9
    open(io->TOML.print(io,report;sorted=true),joinpath(directory,"summary.toml"),"w")
    return report
end

function contact_trajectory_difference(a,b; rotation=SMatrix{3,3,Float64}(I),quaternion=SVector(0.,0.,0.,1.))
    maximum(range(0.,min(last(a.t),last(b.t));length=129)) do t
        x,y=a(t).sc[1],b(t).sc[1]
        maximum((norm(rotation*(x.pos-first(a.u).sc[1].pos)-(y.pos-first(b.u).sc[1].pos)),
            norm(rotation*x.att_r-y.att_r),norm(rotation*x.att_v-y.att_v)/.25,
            norm(rotation*x.vel-y.vel)/.25,norm(x.ω-y.ω)/.25,norm(x.att_ω-y.att_ω)/.25,
            contact_angle(contact_quaternion_product(quaternion,SVector{4,Float64}(x.q)),SVector{4,Float64}(y.q)),
            contact_angle(contact_quaternion_product(quaternion,SVector{4,Float64}(x.att_q[:,1])),SVector{4,Float64}(y.att_q[:,1]))))
    end
end
