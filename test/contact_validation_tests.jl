module ContactValidationTests
using Test, SpaceAGORA, LinearAlgebra, StaticArrays
using ..SpaceAGORAMuJoCo
include("../examples/bus_bumper_fixture.jl")
include("../examples/contact_validation.jl")
const C=SpaceAGORA.ContactLoadServices
const RESULTS=Dict{String,Any}()
run_case(;contact=BusBumperContact(),kwargs...) = begin
    args=bus_bumper_configuration(;kwargs...)
    sol=run_simulation(C.with_contact_model(args,contact);return_solution=true,visualization=false)
    (;args,sol,contact)
end
include("../examples/contact_validation_report.jl")
trajectory_difference(args...;kwargs...)=contact_trajectory_difference(args...;kwargs...)

@testset "stable orientation error near zero" begin
    q=SVector(0.,0.,0.,1.)
    @test contact_angle(q,-q)==0.
    @test contact_angle(q,SVector(sin(5e-10),0.,0.,cos(5e-10)))≈1e-9 rtol=1e-12
    @test contact_angle(q,SVector(sin(pi/4),0.,0.,cos(pi/4)))≈pi/2
end

@testset "forced contact rejection and accepted-trajectory replay" begin
    a=run_case(duration=.25,gap=1e-5,h=2.0^-10)
    b=run_case(duration=.25,gap=1e-5,h=2.0^-10)
    reference=run_case(duration=.25,gap=1e-5,h=2.0^-14,tolerance=1e-12)
    @test a.sol.stats.nreject>0
    @test a.sol.u==b.sol.u && a.sol.t==b.sol.t
    error=trajectory_difference(a.sol,reference.sol)
    @test error<1e-3
    RESULTS["rejection_replay"]=Dict("rejected_steps"=>a.sol.stats.nreject,"scaled_trajectory_error"=>error)
end

@testset "rotated physical contact trajectory" begin
    contact=BusBumperContact(friction=.4)
    a=run_case(;contact,duration=.25,offset=.2,tangent=.15)
    axis=normalize(SVector(.2,-.3,.5)); angle=.8
    q=SVector((axis*sin(angle/2))...,cos(angle/2)); R=contact_rotation(q)
    b=run_case(;contact,duration=.25,offset=.2,tangent=.15,attitude=q,origin=R*SVector(7e6,0.,0.))
    error=trajectory_difference(a.sol,b.sol;rotation=R,quaternion=q)
    @test error<1e-3
    RESULTS["rotation"]=Dict("angle_rad"=>angle,"axis"=>collect(axis),"scaled_trajectory_error"=>error)
end

@testset "independent core tolerance and native parameter sweeps" begin
    tolerances=(1e-6,1e-8,1e-10,1e-12)
    runs=[run_case(duration=.25,tolerance=tol,h=2.0^-10) for tol in tolerances]
    errors=[trajectory_difference(r.sol,last(runs).sol) for r in runs[1:end-1]]
    @test last(errors)<1e-3
    @test last(errors)<first(errors)
    RESULTS["core_tolerances"]=Dict("tolerances"=>collect(tolerances),"scaled_trajectory_errors"=>errors,"maximum_step_s"=>2.0^-10)
    # Ref-safe remains inactive: changing a native timestep must not create a
    # second integrator or change this positive-solref contact law.
    native_steps=(2.0^-10,2.0^-12,2.0^-14)
    step_runs=[run_case(duration=.25,contact=BusBumperContact(timestep=h)) for h in native_steps]
    step_errors=[trajectory_difference(r.sol,last(step_runs).sol) for r in step_runs[1:end-1]]
    @test all(<(1e-3),step_errors)
    RESULTS["native_timesteps"]=Dict("native_steps_s"=>collect(native_steps),"scaled_trajectory_errors"=>step_errors)
    solve_tols=(1e-12,1e-13,1e-14)
    solve_runs=[run_case(duration=.25,offset=.2,tangent=.15,contact=BusBumperContact(friction=.4,tolerance=tol)) for tol in solve_tols]
    solve_errors=[trajectory_difference(r.sol,last(solve_runs).sol) for r in solve_runs[1:end-1]]
    @test all(<(1e-3),solve_errors)
    RESULTS["native_solver_tolerances"]=Dict("tolerances"=>collect(solve_tols),"scaled_trajectory_errors"=>solve_errors)
end

@testset "off-center friction and damping energy-work ledgers" begin
    for (name,offset,tangent,mu,damping) in (("sliding",.2,.4,.1,0.),("damped",.2,.15,.4,.2),("slow",.2,.01,.8,0.),("grazing",.42,.05,.4,0.))
        r=run_case(duration=.25,offset=offset,tangent=tangent,translation_damping=damping,contact=BusBumperContact(friction=mu))
        coarse=contact_trajectory_ledger(r.args,r.sol,r.contact;sample_step=2.0^-10)
        fine=contact_trajectory_ledger(r.args,r.sol,r.contact;sample_step=2.0^-11)
        m=fine.metrics
        @test m["maximum_penetration_m"]>0
        @test last(fine.rows).gap>0
        @test m["energy_work_residual_J"]/1.3125<1e-5
        @test abs(m["contact_work_J"]-coarse.metrics["contact_work_J"])/1.3125<1e-6
        @test m["momentum_residual_kg_m_s"]/5.25<1e-7
        @test m["angular_momentum_residual_kg_m2_s"]/5.25<1e-7
        @test m["force_balance_residual_N"]<1e-10
        @test m["moment_balance_residual_Nm"]<1e-10
        @test m["mechanical_energy_change_J"]<0
        @test m["maximum_damping_power_W"]<=1e-12
        @test m["power_projection_residual_W"]<1e-10
        if name=="slow"
            @test_broken m["friction_work_J"]<=1e-10 # Issue #4: native soft-friction counterexample.
        else
            @test m["friction_work_J"]<=1e-10
        end
        @test m["incoming_normal_speed_m_s"]>0 && m["outgoing_normal_speed_m_s"]>0
        @test m["maximum_cone_ratio"]<=1+1e-8
        if name=="sliding"
            @test_broken m["maximum_sliding_friction_power_W"]<=1e-10 # Qualification remains blocked.
        else
            @test m["maximum_sliding_friction_power_W"]<=1e-10
        end
        name=="sliding" && @test m["sliding_samples"]>0
        name=="slow" && @test m["low_slip_interior_samples"]>0
        RESULTS[name]=m
    end
end

@testset "regularization sensitivity is a physical model sweep" begin
    values=Dict{String,Any}[]
    for tau in (.005,.01,.02)
        r=run_case(duration=.25,h=2.0^-13,contact=BusBumperContact(time_constant=tau))
        m=contact_trajectory_ledger(r.args,r.sol,r.contact).metrics
        @test m["maximum_penetration_m"]>0
        @test m["energy_work_residual_J"]/1.3125<1e-5
        @test m["mechanical_energy_change_J"]<0
        @test m["incoming_normal_speed_m_s"]>0 && m["outgoing_normal_speed_m_s"]>0
        m["time_constant_s"]=tau
        push!(values,m)
    end
    RESULTS["physical_regularization"]=values
end

@testset "stricter native solve retains the physical residual budget" begin
    for tol in (1e-13,1e-14)
        r=run_case(duration=.25,h=2.0^-11,offset=.2,tangent=.15,translation_damping=.2,
            contact=BusBumperContact(friction=.4,tolerance=tol))
        m=contact_trajectory_ledger(r.args,r.sol,r.contact).metrics
        @test m["maximum_penetration_m"]>0
        @test m["energy_work_residual_J"]/1.3125<1e-5
        @test m["power_projection_residual_W"]<1e-10
    end
end

if haskey(ENV,"CONTACT_VALIDATION_TEST_REPORT")
    path=ENV["CONTACT_VALIDATION_TEST_REPORT"]
    mkpath(dirname(path))
    open(io->TOML.print(io,RESULTS;sorted=true),path,"w")
end
end
