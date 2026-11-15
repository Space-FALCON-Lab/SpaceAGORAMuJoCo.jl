module CoulombContactTests
using Test, LinearAlgebra, StaticArrays
using ..SpaceAGORAMuJoCo
const C=SpaceAGORAMuJoCo.CL
include("../examples/bus_bumper_fixture.jl")
include("../examples/contact_validation.jl")
include("../examples/coulomb_validation.jl")
const GEOMETRIES=(ContactGeometry(1,:box,(.5,.4,.3)),ContactGeometry(2,:sphere,(.1,)))
@testset "accepted Coulomb hold, arrest, breakaway and re-sticking" begin
    for (name,tangent,rest,duration) in ((:hold,0.,.001,.05),(:arrest,.02,.001,.05),(:reversal,.02,.02,.5))
        contact=CoulombContact(GEOMETRIES,((1,2),))
        args=bus_bumper_configuration(;duration,tangent,rest_tangent=rest,rest_gap=-.01,gap=-.001,normal_speed=0.,origin=SVector(0.,0.,0.))
        sol=run_simulation(C.with_contact_model(args,contact);return_solution=true,visualization=false)
        ledger=coulomb_trajectory_ledger(args,sol,contact);m=ledger.metrics
        @test m["sticking_samples"]>10
        @test m["maximum_sticking_speed_m_s"]<1e-10
        @test m["sticking_slip_distance_m"]<1e-10
        @test m["maximum_friction_power_W"]<=1e-10
        @test m["maximum_cone_ratio"]<=1+1e-10
        @test m["friction_work_J"]<=1e-10
        @test m["projection_work_J"]<=1e-12
        @test abs(m["projection_work_J"])<1e-10
        @test m["energy_work_residual_J"]<1.3125e-5
        @test m["momentum_residual_kg_m_s"]<5.25e-7
        @test m["angular_momentum_residual_kg_m2_s"]<5.25e-7
        transitions=m["transitions"]
        @test first(transitions)["sticking"]
        @test (name==:hold ? first(transitions)["time_s"]==0. : first(transitions)["time_s"]>0.)
        if name==:reversal
            @test count(x->!x["sticking"],transitions)>=2
            @test count(x->x["sticking"],transitions)>=3
        else
            @test length(transitions)==1
        end
    end
end
@testset "trial queries cannot commit contact modes" begin
    bodies=(C.ContactBody(20.,Diagonal([5/3,34/15,41/15])),C.ContactBody(1.,.004Matrix{Float64}(I,3,3)))
    rt=C.contact_prepare(CoulombContact(GEOMETRIES,((1,2),)),bodies)
    z=SVector(0.,0.,0.);q=SVector(0.,0.,0.,1.)
    states=(C.ContactState(z,z,q,z,z,z),C.ContactState(SVector(.599,0.,0.),z,q,z,SVector(-2.,.1,0.),z))
    try
        C.contact_accept!(rt,states,0.)
        @test rt.sticking
        before=copy(rt.transitions);load=C.contact_evaluate!(rt,states,0.)
        separated=(states[1],C.ContactState(SVector(2.,0.,0.),z,q,z,z,z))
        @test all(w->iszero(w.force_n),C.contact_evaluate!(rt,separated,1.))
        C.contact_event_value(rt,separated,-1.)
        @test rt.sticking && rt.transitions==before
        @test C.contact_evaluate!(rt,states,0.)==load
    finally
        C.contact_close!(rt)
    end
end
@testset "zero-speed departure removes transverse numerical noise passively" begin
    bodies=(C.ContactBody(20.,Diagonal([5/3,34/15,41/15])),C.ContactBody(1.,.004Matrix{Float64}(I,3,3)))
    rt=C.contact_prepare(CoulombContact(GEOMETRIES,((1,2),)),bodies)
    z=SVector(0.,0.,0.);q=SVector(0.,0.,0.,1.)
    try
        for angle in range(0.,2pi;length=17)
            states=(C.ContactState(z,z,q,z,z,z),C.ContactState(SVector(.59998,0.,0.),SVector(0.,1e-9cos(angle),1e-9sin(angle)),q,z,SVector(-2.,10.,0.),z))
            query=SpaceAGORAMuJoCo._coulomb_query(rt,states,0.)
            @test query.departure && query.margin<0
            corrections=C.contact_accept!(rt,states,0.)
            after=map(states,corrections) do s,c
                C.ContactState(s.position_m,s.velocity_m_s+c.linear_world_m_s,s.quaternion,
                    s.angular_velocity_body_rad_s+c.angular_body_rad_s,s.smooth_force_n,s.smooth_torque_nm)
            end
            result=SpaceAGORAMuJoCo._coulomb_query(rt,after,0.)
            @test dot(result.f[2:3],result.velocity)<=1e-20
            @test norm(sum(b.mass_kg*c.linear_world_m_s for (b,c) in zip(bodies,corrections)))<1e-20
            @test norm(sum(cross(s.position_m,b.mass_kg*c.linear_world_m_s)+b.inertia_body_kg_m2*c.angular_body_rad_s for (b,s,c) in zip(bodies,states,corrections)))<1e-20
            kinetic(ss)=sum((b.mass_kg*sum(abs2,s.velocity_m_s)+dot(s.angular_velocity_body_rad_s,b.inertia_body_kg_m2*s.angular_velocity_body_rad_s))/2 for (b,s) in zip(bodies,ss))
            @test kinetic(after)<=kinetic(states)+1e-30
            @test !rt.sticking && isempty(rt.transitions)
        end
    finally
        C.contact_close!(rt)
    end
end
end
