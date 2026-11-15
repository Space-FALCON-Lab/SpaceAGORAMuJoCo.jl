module CompliantContactTests
using Test, LinearAlgebra, StaticArrays, SpaceAGORA
using ..SpaceAGORAMuJoCo
const C=SpaceAGORA.ContactLoadServices
const B=SpaceAGORAMuJoCo.Binding
const Z=SVector(0.,0.,0.)
const Q=SVector(0.,0.,0.,1.)
include("../examples/bus_bumper_fixture.jl")
include("../examples/contact_measurements.jl")
function rotation(q)
    x,y,z,w=q
    return [1-2(y*y+z*z) 2(x*y-z*w) 2(x*z+y*w);
    2(x*y+z*w) 1-2(x*x+z*z) 2(y*z-x*w); 2(x*z-y*w) 2(y*z+x*w) 1-2(x*x+y*y)]
end
const BODIES=(C.ContactBody(20.,Diagonal([5/3,34/15,41/15])),C.ContactBody(1.,.004Matrix{Float64}(I,3,3)))
state(r;v=Z,q=Q,w=Z,f=Z,tau=Z)=C.ContactState(SVector{3,Float64}(r),v,q,w,f,tau)

@testset "production provider purity, projection, separation and refusals" begin
    SpaceAGORA.Conformance.test_service_facade(C;version=v"0.2.0")
    for c in (BusBumperContact(friction=-1.),BusBumperContact(time_constant=1e-6),
            BusBumperContact(impedance=1.),BusBumperContact(slip_speed=0.))
        @test_throws ArgumentError C.contact_preflight(c,BODIES)
    end
    for friction in (0.,.4)
        rt=C.contact_prepare(BusBumperContact(friction=friction),BODIES)
        other=C.contact_prepare(BusBumperContact(friction=friction),BODIES)
        try
            a=(state(Z),state((.599,.2,0.);v=SVector(-.2,.3,0.)))
            loads=C.contact_evaluate!(rt,a,.2)
            @test norm(loads[2].force_n)>0
            @test loads[1].force_n+loads[2].force_n≈Z atol=1e-13
            @test loads[1].torque_nm+loads[2].torque_nm+cross(a[2].position_m,loads[2].force_n)≈Z atol=1e-13
            constraint=copy(B.qfrc_constraint(rt.model,rt.data))
            for i in 1:2
                @test constraint[6(i-1)+1:6(i-1)+3]≈loads[i].force_n atol=1e-12
                @test constraint[6(i-1)+4:6(i-1)+6]≈loads[i].torque_nm atol=1e-12
            end
            separated=(a[1],state((2.,.2,0.)))
            @test all(w->iszero(w.force_n)&&iszero(w.torque_nm),C.contact_evaluate!(rt,separated,1.))
            @test C.contact_evaluate!(rt,a,.2)==loads
            @test C.contact_evaluate!(other,a,.2)==loads
            @test C.contact_evaluate!(rt,a,.1)==loads
            @test B.data_time(rt.data)==.1
            @test_throws ArgumentError C.contact_evaluate!(rt,(state((NaN,0.,0.)),a[2]),0.)
            @test_throws ArgumentError C.contact_evaluate!(rt,(state(Z;q=2Q),a[2]),0.)
        finally
            C.contact_close!(rt);C.contact_close!(other)
        end
        @test C.contact_close!(rt)===nothing
        @test_throws Exception C.contact_evaluate!(rt,(state(Z),state((.599,0.,0.))),0.)
    end
end

@testset "physical full inertia and spinning free acceleration" begin
    Ibody=@SMatrix [2. .1 .2; .1 2.4 -.15; .2 -.15 2.8]
    bodies=(C.ContactBody(20.,Ibody),BODIES[2])
    rt=C.contact_prepare(BusBumperContact(),bodies)
    try
        a=(state(Z;q=normalize(SVector(.1,.2,.3,.8)),w=SVector(.4,-.3,.2),
                f=SVector(1.,-2.,3.),tau=SVector(.2,-.3,.4)),
            state((2.,0.,0.);q=normalize(SVector(-.2,.3,.1,.7)),w=SVector(.2,.4,-.3),
                f=SVector(-.3,.2,-.1),tau=SVector(-.2,.3,-.4)))
        @test all(w->iszero(w.force_n)&&iszero(w.torque_nm),C.contact_evaluate!(rt,a,0.))
        acceleration=B.qacc_smooth(rt.model,rt.data)
        for i in 1:2
            s=a[i]; inertia=bodies[i].inertia_body_kg_m2
            @test acceleration[6(i-1)+1:6(i-1)+3]≈s.smooth_force_n/bodies[i].mass_kg atol=1e-13
            @test acceleration[6(i-1)+4:6(i-1)+6]≈inertia\(rotation(s.quaternion)'*s.smooth_torque_nm-cross(s.angular_velocity_body_rad_s,inertia*s.angular_velocity_body_rad_s)) atol=1e-12
        end
    finally
        C.contact_close!(rt)
    end
end

@testset "engine head-on contact, recoil and repeatability" begin
    args=C.with_contact_model(bus_bumper_configuration(duration=.25),BusBumperContact())
    sol=run_simulation(args;return_solution=true,visualization=false)
    repeat=run_simulation(args;return_solution=true,visualization=false)
    @test sol.u==repeat.u && sol.t==repeat.t
    gaps=[u.sc[1].att_r[1,1]-.6 for u in sol.u]
    @test minimum(gaps)<0
    @test last(gaps)>0
    @test any(u->norm(u.sc[1].vel-first(sol.u).sc[1].vel)>.001,sol.u)
    momenta=[21SVector{3,Float64}(u.sc[1].vel)+SVector{3,Float64}(u.sc[1].att_v[:,1]) for u in sol.u]
    @test maximum(norm,p-first(momenta) for p in momenta)<1e-7*5.25
end
@testset "head-on refinement and independent scalar/work ledgers" begin
    sols=[run_simulation(C.with_contact_model(bus_bumper_configuration(h=h,duration=.25),BusBumperContact());
        return_solution=true,visualization=false) for h in (2.0^-10,2.0^-11,2.0^-12,2.0^-13)]
    reports=headon_measurements.(sols)
    reference=scalar_headon_reference(.25)
    @test maximum(abs.((last(reports)["final_relative_state"]-reference)./[1.,.25]))<1e-3
    @test maximum(abs.((reports[end]["final_relative_state"]-reports[end-1]["final_relative_state"])./[1.,.25]))<1e-3
    @test abs(reports[end]["normal_impulse_N_s"]-reports[end-1]["normal_impulse_N_s"])/5.25<1e-3
    @test reports[end]["momentum_residual_kg_m_s"]/5.25<1e-7
    @test reports[end]["energy_work_residual_J"]/1.3125<1e-5
    finer=headon_measurements(last(sols);sample_step=2.0^-14)
    @test abs(finer["net_contact_work_J"]-reports[end]["net_contact_work_J"])/1.3125<1e-6
    @test finer["energy_work_residual_J"]/1.3125<1e-5
    @test finer["mechanical_energy_change_J"]<0
end

@testset "off-center frictional recoil and Galilean invariance" begin
    base=bus_bumper_configuration(duration=.25,offset=.2,tangent=.15)
    contact=BusBumperContact(friction=.4)
    sol=run_simulation(C.with_contact_model(base,contact);return_solution=true,visualization=false)
    boosted=bus_bumper_configuration(duration=.25,offset=.2,tangent=.15,
        origin=SVector(7e6+1000.,200.,-100.),boost=SVector(10.,-5.,2.))
    other=run_simulation(C.with_contact_model(boosted,contact);return_solution=true,visualization=false)
    inertia=base.dynamics_model.spacecraft[1].inertia_tensor
    p0=first(sol.u).sc[1].pos
    function invariants(s)
        r=SVector{3,Float64}(s.pos-p0); a=SVector{3,Float64}(s.att_r[:,1])
        v=SVector{3,Float64}(s.vel); av=v+SVector{3,Float64}(s.att_v[:,1])
        P=20v+av
        H=cross(r,20v)+cross(r+a,av)+rotation(s.q)*(inertia*s.ω)+rotation(s.att_q[:,1])*(.004s.att_ω[:,1])
        return P,H
    end
    p,h=invariants(first(sol.u).sc[1])
    @test maximum(norm(invariants(u.sc[1])[1]-p) for u in sol.u)/5.25<1e-7
    @test maximum(norm(invariants(u.sc[1])[2]-h) for u in sol.u)/5.25<1e-7
    @test maximum(norm(u.sc[1].ω) for u in sol.u)>1e-4
    for t in range(0.,.25;length=33)
        a,b=sol(t).sc[1],other(t).sc[1]
        @test norm(a.att_r-b.att_r)<1e-3
        @test norm(a.att_v-b.att_v)/.25<1e-3
        @test norm(a.q-b.q)<1e-3
    end
end

end
