module RegularizedFrictionTests
using Test, LinearAlgebra, StaticArrays
using ..SpaceAGORAMuJoCo
const C=SpaceAGORAMuJoCo.CL
const B=SpaceAGORAMuJoCo.Binding
const solve=SpaceAGORAMuJoCo._regularized_contact_force
const residual=SpaceAGORAMuJoCo._regularized_contact_residual
include("../examples/friction_creep.jl")

@testset "coupled passive law against high-precision equation solve" begin
    W=[1.4 .2 -.1; .2 2.1 .3; -.1 .3 1.8]; R=.3
    for mu in (0.,.1,.8,1.2), vs in (.00025,.001,.004),
            speed in (0.,1e-12,.1vs,.5vs,vs,2vs,.4), angle in (0.,.7,2.3)
        v=speed.*[cos(angle),sin(angle)]
        f=solve(W,R,-8.5,v,mu,vs)
        # Solve all three constitutive equations simultaneously in BigFloat;
        # independent of the production scalar elimination and force certificate.
        vb=BigFloat.(v); d=-BigFloat(mu).*vb/max(BigFloat(vs),norm(vb))
        H=BigFloat[W[1,1]+R W[1,2] W[1,3]; -d[1] 1 0; -d[2] 0 1]
        reference=H\BigFloat[8.5,0,0]
        @test norm(f-Float64.(reference))<1e-12
        @test dot(f[2:3],v)<=0
        @test norm(f[2:3])<=mu*f[1]+1e-13
        @test residual(W,R,-8.5,v,mu,vs,f)<1e-12
        @test solve(W,R,2.,v,mu,vs)==zeros(3)
    end
    f=solve(W,R,-8.5,[.1,0.],.4,.001)
    @test residual(W,R,-8.5,[.1,0.],.4,.001,f+[.01,0.,0.])>1e-3
    @test residual(W,R,-8.5,[.1,0.],.4,.001,f+[0.,.01,0.])>1e-3
    @test_throws ArgumentError solve([1. .9 0.; .9 1. 0.; 0. 0. 1.],.1,-1.,[1.,0.],2.,.001)
end

@testset "bounded creep and transition to sliding against analytic motion" begin
    report=friction_creep_report()
    @test report["qualified"]
    for r in report["cases"]
        @test r["maximum_friction_power_W"]<=1e-12
        @test r["velocity_error_m_s"]<1e-7
        @test r["displacement_error_m"]<1e-7
        @test r["energy_work_residual_J"]<1e-8
        if r["subcritical"]
            @test r["maximum_speed_m_s"]<=r["creep_speed_budget_m_s"]
            @test abs(r["displacement_m"])<=r["creep_displacement_budget_m"]
        else
            @test abs(r["final_velocity_m_s"])>r["slip_speed_m_s"]
        end
    end
    for alpha in (-.75,.75)
        rows=filter(r->r["load_fraction"]==alpha,report["cases"])
        @test all(diff([abs(r["displacement_m"]) for r in rows]).>0)
    end
    for alpha in (-1.25,1.25)
        fine=friction_creep_case(alpha,.001;step_fraction=.1)
        coarse=friction_creep_case(alpha,.001;step_fraction=.2)
        @test fine["velocity_error_m_s"]<coarse["velocity_error_m_s"]
        @test fine["displacement_error_m"]<coarse["displacement_error_m"]
    end
end

@testset "native slip reversal, offset and smooth-load sweep" begin
    bodies=(C.ContactBody(20.,Diagonal([5/3,34/15,41/15])),C.ContactBody(1.,.004Matrix{Float64}(I,3,3)))
    Z=SVector(0.,0.,0.); Q=SVector(0.,0.,0.,1.)
    for mu in (.1,.4,.8)
        c=BusBumperContact(friction=mu); rt=C.contact_prepare(c,bodies)
        try
            for offset in (-.3,0.,.3), slip in (-.4,-.01,-.001,0.,.001,.01,.4),
                    vn in (-.2,0.,.2), external in (-2.,0.,2.)
                states=(C.ContactState(Z,Z,Q,Z,Z,Z),
                    C.ContactState(SVector(.599,offset,0.),SVector(vn,slip,0.),Q,Z,SVector(-1.,external,0.),Z))
                loads=C.contact_evaluate!(rt,states,0.)
                f=B.efc_force(rt.data); v=B.efc_vel(rt.data)
                @test dot(f[2:3],v[2:3])<=1e-12
                @test norm(f[2:3])<=mu*f[1]+1e-12
                @test f[1]>=0
                power=sum(dot(w.force_n,s.velocity_m_s) for (w,s) in zip(loads,states))
                @test abs(power-dot(f,v))<1e-10
            end
        finally
            C.contact_close!(rt)
        end
    end
end
end
