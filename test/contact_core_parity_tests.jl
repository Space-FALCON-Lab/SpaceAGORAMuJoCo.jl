module ContactCoreParityTests
using Test, LinearAlgebra, StaticArrays, SpaceAGORA
using ..ContactFeasibilityTests: ContactHarness, contact_query, probe_state, probe_rotation

@testset "contact feasibility: core/native free-acceleration parity" begin
    h=ContactHarness()
    try
        q,v,w=probe_state(x=2.0)
        q[4:7] .= normalize([.8,.1,.2,.3])
        q[11:14] .= normalize([.7,-.2,.3,.1])
        v .= [.1,.2,.3,.4,-.3,.2,-.1,.3,.2,.2,.4,-.3]
        w[:,1] .= [1.,-2.,3.,.2,-.3,.4]
        w[:,2] .= [-.3,.2,-.1,-.2,.3,-.4]
        bodies=[SpaceAGORA.CompliantBody(Symbol(:body,i),h.mass_matrix.diag[6(i-1)+1],
            SMatrix{3,3,Float64}(Diagonal(h.mass_matrix.diag[6(i-1)+4:6(i-1)+6]))) for i in 1:2]
        model=SpaceAGORA.CompliantMultibodyModel(bodies,SpaceAGORA.CompliantJoint[],
            SVector(0.,0.,0.),SVector(0.,0.,0.,1.))
        positions=[q[7(i-1)+1:7(i-1)+3] for i in 1:2]
        quats=[q[[7(i-1)+5,7(i-1)+6,7(i-1)+7,7(i-1)+4]] for i in 1:2]
        x=SpaceAGORA.compliant_state_vector(positions,quats;
            velocities=[v[6(i-1)+1:6(i-1)+3] for i in 1:2],
            angular_velocities=[v[6(i-1)+4:6(i-1)+6] for i in 1:2])
        torques=[probe_rotation(q[7(i-1)+4:7(i-1)+7])'*w[4:6,i] for i in 1:2]
        dx=SpaceAGORA.compliant_multibody_dynamics(model,x;
            external_forces_world=[w[1:3,i] for i in 1:2],external_torques_body=torques)
        native=contact_query(h,q,v,w)
        @test isempty(native.f)
        for i in 1:2
            @test dx[13(i-1)+8:13(i-1)+10] ≈ native.a0[6(i-1)+1:6(i-1)+3] atol=1e-13
            @test dx[13(i-1)+11:13(i-1)+13] ≈ native.a0[6(i-1)+4:6(i-1)+6] atol=1e-12
        end
    finally
        close(h)
    end
end
end
