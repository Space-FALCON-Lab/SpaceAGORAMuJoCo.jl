module ContactFeasibilityTests
using Test, LinearAlgebra
using ..SpaceAGORAMuJoCo: Binding
include("support/contact_feasibility.jl")
const RESULTS=Dict{String,Any}()

@testset "contact binding: record stride, copied geometry and sparse refusal" begin
    xml="""<mujoco><option gravity="0 0 0" jacobian="dense"/>
      <default><geom condim="1"/></default><worldbody>
      <geom type="plane" size="2 2 .1"/>
      <body pos="-.3 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
      <body pos=".3 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
      </worldbody></mujoco>"""
    m=Binding.load_xml_string(xml);d=Binding.MjData(m)
    try
        Binding.forward!(m,d)
        @test Binding.ncon(d)==2
        contacts=[Binding.contact(d,k) for k in 1:2]
        @test sort([c.pos[1] for c in contacts]) ≈ [-.3,.3]
        @test Set(c.geom for c in contacts)==Set([(0,1),(0,2)])
        @test sort([c.efc_address for c in contacts])==[0,1]
        @test all(c.dist ≈ -.01 for c in contacts)
        @test all(Binding.contact_force!(zeros(6),m,d,k)[1]>0 for k in 1:2)
        Binding.qpos(m,d)[3]=.3;Binding.qpos(m,d)[10]=.3
        Binding.forward!(m,d)
        @test Binding.ncon(d)==0
        @test all(c.dist ≈ -.01 for c in contacts) # previous copies survived arena reuse
    finally
        close(d);close(m)
    end
    @test_throws InvalidStateException Binding.contact(d,1)
    sparse=Binding.load_xml_string(replace(xml,"jacobian=\"dense\""=>"jacobian=\"sparse\""))
    sd=Binding.MjData(sparse)
    try
        Binding.forward!(sparse,sd)
        @test_throws ArgumentError Binding.constraint_jacobian(sparse,sd)
    finally
        close(sd);close(sparse)
    end
end

@testset "contact feasibility: free dynamics, state purity and scalar response" begin
    h=ContactHarness()
    try
        q,v,w=probe_state(x=.7)
        # A rotated anisotropic bus, with spin and physical COM force/torque.
        q[4:7] .= [cos(.2),0,0,sin(.2)]
        v[4:6] .= [.2,-.3,.4]; w[:,1] .= [1,2,3,.1,.2,-.3]
        w[:,2] .= [-1,.5,.2,.02,-.01,.03]
        a=contact_query(h,q,v,w;time=2.0)
        @test isempty(a.f)
        @test a.loads == zeros(6,2)
        for i in 1:2
            k=6(i-1); R=probe_rotation(q[7(i-1)+4:7(i-1)+7])
            Ibody=h.mass_matrix.diag[k+4:k+6]; omega=v[k+4:k+6]
            @test a.acc[k+1:k+3] ≈ w[1:3,i]/h.mass_matrix.diag[k+1] atol=1e-13
            @test a.acc[k+4:k+6] ≈ (R'*w[4:6,i]-cross(omega,Ibody.*omega))./Ibody atol=1e-12
        end
        q,v,w=probe_state(); v[7]=-.25
        initial=(copy(q),copy(v),copy(w))
        a=contact_query(h,q,v,w;time=.75)
        @test (q,v,w)==initial
        @test Binding.data_time(h.data)==.75
        @test Binding.qpos(h.model,h.data)==q
        @test Binding.qvel(h.model,h.data)==v
        @test length(a.contacts)==1
        c=only(a.contacts)
        # MuJoCo orders this pair sphere first, independently of MJCF pair order.
        @test c.geom==(1,0)
        @test c.dim==1 && c.efc_address==0
        @test c.dist ≈ -.001 atol=1e-15
        @test collect(c.pos) ≈ [.4995,0,0] atol=1e-14
        @test collect(c.frame[1:3]) ≈ [-1,0,0] atol=1e-14
        # Independent scalar law: d=.9, tau=.01, damping ratio 1; diagexact enabled.
        A=1/20+1.0; d=.9; tau=.01
        aref=-2/(d*tau)*(-.25)-1/(d*tau^2)*(-.001)
        expected=max(0.0,aref/(A/d))
        @test only(a.diagA) ≈ A atol=1e-13
        @test only(a.R) ≈ (1-d)/d*A atol=1e-13
        @test only(a.aref) ≈ aref atol=1e-11
        @test only(a.f) ≈ expected atol=1e-10
        RESULTS["scalar"] = Dict("expected_force_N"=>expected,"native_force_N"=>only(a.f),
            "force_error_N"=>abs(only(a.f)-expected),"residual_N"=>a.residual,
            "effective_mass_kg"=>1/A)
        @test a.loads[1,2] ≈ expected atol=1e-10
        @test a.loads[1,1] ≈ -expected atol=1e-10
        @test norm(a.generalized-a.J'*a.f) < 1e-12
        @test norm(a.acc-a.a0-h.mass_matrix\a.generalized) < 1e-11
        # Smooth force must affect contact, but must not be included in returned contact loads.
        w[1,2]=-2.0
        loaded=contact_query(h,q,v,w)
        @test only(loaded.f) ≈ (aref+2)/(A/d) atol=1e-10
        @test loaded.loads[1,2] ≈ only(loaded.f) atol=1e-12
        # A-B-A, out-of-order times, then an identical time with different geometry.
        w .= 0
        qb,vb,wb=probe_state(x=.8)
        @test isempty(contact_query(h,qb,vb,wb;time=-2).f)
        again=contact_query(h,q,v,w;time=.75)
        @test again==a
        @test isempty(contact_query(h,qb,vb,wb;time=.75).f)
        @test contact_query(h,q,v,w;time=.75)==a
        # Separating quickly while overlapping gives zero unilateral force.
        v[7]=2
        @test norm(contact_query(h,q,v,w).f) < 1e-12
        bad=copy(q); bad[1]=NaN
        @test_throws ArgumentError contact_query(h,bad,v,w)
        @test_throws DimensionMismatch contact_query(h,q,v,zeros(6,1))
        bad=copy(q); bad[4]=2
        @test_throws ArgumentError contact_query(h,bad,v,w)
        @test_throws BoundsError Binding.contact(h.data,0)
        @test_throws BoundsError Binding.contact_force!(zeros(6),h.model,h.data,2)
        @test_throws DimensionMismatch Binding.contact_force!(zeros(5),h.model,h.data,1)
    finally
        close(h)
    end
    @test_throws InvalidStateException Binding.ncon(h.data)
    @test_throws InvalidStateException Binding.qacc(h.model,h.data)
end

@testset "contact feasibility: off-center moments, friction and isolation" begin
    for dim in (1,3)
        h=ContactHarness(;dim); other=ContactHarness(;dim)
        try
            q,v,w=probe_state(y=.2); v[7:9] .= [-.25, .8, -.1]
            a=contact_query(h,q,v,w)
            @test a.residual < 1e-9
            # Geometry-derived contact Jacobian (world translation/body angular velocity).
            c=only(a.contacts); basis=reshape(collect(c.frame),3,3)
            J=zeros(dim,12)
            for i in 1:2, k in 1:dim
                sign=Binding.geom_bodyid(h.model)[c.geom[1]+1]==i ? -1 : 1; n=basis[:,k]
                r=q[7(i-1)+1:7(i-1)+3]; R=probe_rotation(q[7(i-1)+4:7(i-1)+7])
                J[k,6(i-1)+1:6(i-1)+3] .= sign.*n
                J[k,6(i-1)+4:6(i-1)+6] .= sign.*(R'*cross(collect(c.pos)-r,n))
            end
            @test a.J ≈ J atol=1e-13
            A=J*(h.mass_matrix\J')
            @test a.diagA[1] ≈ A[1,1] atol=1e-12
            # Elliptic contact regularization shares its normal inertia across
            # tangential rows; diagA is not the full directional inverse mass.
            # Verify that mass independently with native free-acceleration responses.
            for k in 1:dim
                load=zeros(6,2)
                for i in 1:2
                    R=probe_rotation(q[7(i-1)+4:7(i-1)+7])
                    load[1:3,i] .= J[k,6(i-1)+1:6(i-1)+3]
                    load[4:6,i] .= R*J[k,6(i-1)+4:6(i-1)+6]
                end
                forced=contact_query(h,q,v,load)
                @test J*(forced.a0-a.a0) ≈ A[:,k] atol=1e-12
            end
            @test a.diagA[1] ≈ 1+1/20+.2^2/(41/15) atol=1e-12
            generalized=zeros(12)
            moment=zeros(3)
            for i in 1:2
                R=probe_rotation(q[7(i-1)+4:7(i-1)+7]); r=q[7(i-1)+1:7(i-1)+3]
                generalized[6(i-1)+1:6(i-1)+3] .= a.loads[1:3,i]
                generalized[6(i-1)+4:6(i-1)+6] .= R'*a.loads[4:6,i]
                moment .+= cross(r,a.loads[1:3,i])+a.loads[4:6,i]
            end
            @test generalized ≈ a.generalized atol=1e-11
            @test norm(sum(a.loads[1:3,:];dims=2)) < 1e-12
            @test norm(moment) < 1e-12
            RESULTS["contact_dim_$dim"] = Dict("residual_N"=>a.residual,
                "moment_residual_Nm"=>norm(moment),
                "force_projection_error_N"=>norm((generalized-a.generalized)[[1,2,3,7,8,9]]),
                "torque_projection_error_Nm"=>norm((generalized-a.generalized)[[4,5,6,10,11,12]]),
                "normal_inverse_mass_per_kg"=>A[1,1])
            if dim==3
                @test norm(a.f[2:3]) <= .4a.f[1]+1e-10
                @test dot(a.f[2:3],a.velocity[2:3]) < 0
            end
            @test contact_query(other,q,v,w)==a
            qb,vb,wb=probe_state(x=.9)
            contact_query(other,qb,vb,wb)
            @test contact_query(h,q,v,w)==a
            # Rotate the entire physical fixture, including both body attitudes.
            angle=.6; qr=[cos(angle/2),0,0,sin(angle/2)]; G=probe_rotation(qr)
            rotated_q=copy(q);rotated_v=copy(v)
            for i in 1:2
                rotated_q[7(i-1)+1:7(i-1)+3] .= G*q[7(i-1)+1:7(i-1)+3]
                rotated_q[7(i-1)+4:7(i-1)+7] .= qr
                rotated_v[6(i-1)+1:6(i-1)+3] .= G*v[6(i-1)+1:6(i-1)+3]
            end
            rotated=contact_query(h,rotated_q,rotated_v,w)
            @test rotated.loads[1:3,:] ≈ G*a.loads[1:3,:] atol=1e-9
            @test rotated.loads[4:6,:] ≈ G*a.loads[4:6,:] atol=1e-9
            # A constant translation and boost change neither contact geometry nor loads.
            q[1:3] .+= [10,-5,2]; q[8:10] .+= [10,-5,2]
            v[1:3] .+= [2,-3,1]; v[7:9] .+= [2,-3,1]
            shifted=contact_query(h,q,v,w)
            @test shifted.loads ≈ a.loads atol=1e-9
        finally
            close(h);close(other)
        end
    end
    failed=ContactHarness(iterations=0)
    try
        q,v,w=probe_state();v[7]=-.25
        @test_throws ErrorException contact_query(failed,q,v,w)
    finally
        close(failed)
    end
end

@testset "contact feasibility: inertia and native timestep sweeps" begin
    maximum_error=0.0
    for mass in (2.0,20.0,200.0), timestep in (2.0^-10,2.0^-12,2.0^-14)
        h=ContactHarness(;mass,timestep)
        try
            q,v,w=probe_state();v[7]=-.25
            a=contact_query(h,q,v,w)
            expected=(2*.25/(.9*.01)+.001/(.9*.01^2))/((1+1/mass)/.9)
            maximum_error=max(maximum_error,abs(only(a.f)-expected))
            @test only(a.f) ≈ expected atol=1e-10
            @test Binding.body_mass(h.model) ≈ [0,mass,1] atol=1e-14
            @test Binding.body_inertia(h.model)[4:6] ≈ h.mass_matrix.diag[4:6] atol=1e-13
            @test Binding.body_inertia(h.model)[7:9] ≈ fill(.004,3) atol=1e-14
        finally
            close(h)
        end
    end
    RESULTS["sweep"]=Dict("cases"=>9,"maximum_scalar_force_error_N"=>maximum_error,
        "bus_masses_kg"=>[2.,20.,200.],"native_timesteps_s"=>[2.0^-10,2.0^-12,2.0^-14])
end

end
