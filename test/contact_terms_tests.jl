module NativeContactTermsTests
using Test, LinearAlgebra
using ..SpaceAGORAMuJoCo: Binding
include("support/contact_feasibility.jl")

@testset "native contact terms without a force solve or integration" begin
    for dim in (1,3), x in (.599,.8), y in (0.,.2)
        h=ContactHarness(;dim)
        try
            q,v,w=probe_state(;x,y)
            v[1:6].=[.02,-.03,.01,.04,-.03,.02]
            v[7:12].=[-.25,.12,.03,-.02,.01,-.05]
            w[:,1].=[1.,-.2,.3,.1,.02,-.03]
            w[:,2].=[-.4,.3,.2,.01,-.02,.03]
            full=contact_query(h,q,v,w;time=.4)
            m,d=h.model,h.data
            # Start from a solved state: old constraint forces must be cleared.
            Binding.contact_terms!(m,d)
            @test Binding.constraint_jacobian(m,d)==full.J
            @test Binding.efc_R(d)==full.R
            @test Binding.efc_aref(d)==full.aref
            @test Binding.efc_vel(d)==full.velocity
            @test Binding.qacc_smooth(m,d)==full.a0
            @test Binding.qacc(m,d)==full.a0
            @test all(iszero,Binding.efc_force(d))
            @test all(iszero,Binding.qfrc_constraint(m,d))
            @test Binding.qpos(m,d)==q && Binding.qvel(m,d)==v
            @test Binding.data_time(d)==.4
        finally
            close(h)
        end
        @test_throws InvalidStateException Binding.contact_terms!(h.model,h.data)
    end
end
end
