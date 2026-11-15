module PrimitiveContactTests
using Test, LinearAlgebra, StaticArrays
using ..SpaceAGORAMuJoCo
const C=SpaceAGORAMuJoCo.CL
const B=SpaceAGORAMuJoCo.Binding
const Z=SVector(0.,0.,0.);const Q=SVector(0.,0.,0.,1.)
state(r;v=Z,q=Q,w=Z,f=Z,tau=Z)=C.ContactState(SVector{3,Float64}(r),v,q,w,f,tau)
const BODIES=(C.ContactBody(20.,Diagonal([5/3,34/15,41/15])),C.ContactBody(1.,.004Matrix{Float64}(I,3,3)))

@testset "primitive provider agrees with qualified bus–bumper law" begin
    for mu in (0.,.1,.4,.8), offset in (-.2,0.,.2), tangent in (-.2,0.,.2)
        geoms=(ContactGeometry(1,:box,(.5,.4,.3)),ContactGeometry(2,:sphere,(.1,)))
        a=C.contact_prepare(PrimitiveContact(geoms,((1,2),);friction=mu),BODIES)
        b=C.contact_prepare(BusBumperContact(friction=mu),BODIES)
        try
            states=(state(Z),state((.599,offset,0.);v=SVector(-.2,tangent,0.)))
            x=C.contact_evaluate!(a,states,.2);y=C.contact_evaluate!(b,states,.2)
            for i in 1:2
                @test x[i].force_n≈y[i].force_n atol=1e-11
                @test x[i].torque_nm≈y[i].torque_nm atol=1e-11
            end
            separated=(states[1],state((2.,0.,0.)))
            @test all(w->iszero(w.force_n)&&iszero(w.torque_nm),C.contact_evaluate!(a,separated,.4))
            @test C.contact_evaluate!(a,states,.2)==x
        finally
            C.contact_close!(a);C.contact_close!(b)
        end
    end
end

@testset "multiple contacts exchange balanced physical COM wrenches" begin
    bodies=(BODIES...,C.ContactBody(.5,Diagonal([.01,.02,.03])))
    geoms=(ContactGeometry(1,:box,(.5,.4,.3)),ContactGeometry(2,:sphere,(.1,)),
        ContactGeometry(3,:capsule,(.04,.1)))
    contact=PrimitiveContact(geoms,((1,2),(1,3));friction=.4)
    rt=C.contact_prepare(contact,bodies)
    try
        states=(state(Z),state((.599,.2,0.);v=SVector(-.2,.1,0.)),
            state((.539,-.2,0.);v=SVector(-.1,-.2,0.)))
        loads=C.contact_evaluate!(rt,states,0.)
        @test B.ncon(rt.data)>=2
        @test B.geom_distance(rt.model,rt.data,B.name2id(rt.model,5,"g1")+1,B.name2id(rt.model,5,"g2")+1)≈-.001 atol=1e-12
        @test_throws BoundsError B.geom_distance(rt.model,rt.data,0,1)
        @test all(w->norm(w.force_n)>0,loads)
        @test norm(sum(w.force_n for w in loads))<1e-10
        @test norm(sum(w.torque_nm+cross(s.position_m,w.force_n) for (w,s) in zip(loads,states)))<1e-10
        J=B.constraint_jacobian(rt.model,rt.data);f=B.efc_force(rt.data);v=B.efc_vel(rt.data)
        for i in 1:3
            @test loads[i].force_n≈(J'*f)[6(i-1)+1:6(i-1)+3] atol=1e-10
            @test loads[i].torque_nm≈(J'*f)[6(i-1)+4:6(i-1)+6] atol=1e-10
        end
        for k in 1:3:length(f)
            @test f[k]>=0
            @test norm(f[k+1:k+2])<=.4f[k]+1e-12
            @test dot(f[k+1:k+2],v[k+1:k+2])<=1e-12
        end
        @test C.contact_evaluate!(rt,states,0.)==loads
    finally
        C.contact_close!(rt)
    end
end
@testset "primitive geometry refuses ambiguous or invalid inputs" begin
    box=ContactGeometry(1,:box,(.5,.4,.3));sphere=ContactGeometry(2,:sphere,(.1,))
    for (geometries,pairs) in (((box,sphere),((1,2),(2,1))),
            ((box,sphere),((1,3),)),((box,ContactGeometry(1,:sphere,(.1,))),((1,2),)),
            ((box,ContactGeometry(2,:mesh,(.1,))),((1,2),)),
            ((box,ContactGeometry(2,:sphere,(-.1,))),((1,2),)),
            ((box,ContactGeometry(2,:sphere,(.1,);quaternion=(0.,0.,0.,2.))),((1,2),)))
        @test_throws ArgumentError C.contact_prepare(PrimitiveContact(geometries,pairs),BODIES)
    end
end
@testset "all primitive pairings preserve mass and world-frame wrench projection" begin
    shapes=(:sphere=>(.1,),:box=>(.1,.1,.1),:capsule=>(.1,.1))
    bodies=(C.ContactBody(2.,[.05 .002 .003;.002 .06 .001;.003 .001 .07]),BODIES[2])
    axis=normalize(SVector(1.,2.,3.));q=SVector((sin(.2)*axis)...,cos(.2));R=SpaceAGORAMuJoCo._contact_rotation(q)
    for (ka,sa) in shapes,(kb,sb) in shapes
        rt=C.contact_prepare(PrimitiveContact((ContactGeometry(1,ka,sa),ContactGeometry(2,kb,sb)),((1,2),);friction=.2),bodies)
        try
            states=(state(Z;q=q),state(R*SVector(.199,0.,0.);q=q,v=R*SVector(-.02,.01,.01)))
            loads=C.contact_evaluate!(rt,states,0.)
            @test B.ncon(rt.data)>0
            J=B.constraint_jacobian(rt.model,rt.data);f=B.efc_force(rt.data)
            @test norm(sum(w.force_n for w in loads))<1e-10
            @test norm(sum(w.torque_nm+cross(s.position_m,w.force_n) for (w,s) in zip(loads,states)))<1e-10
            for i in 1:2
                @test loads[i].force_n≈(J'*f)[6(i-1)+1:6(i-1)+3] atol=1e-10
                @test R'*loads[i].torque_nm≈(J'*f)[6(i-1)+4:6(i-1)+6] atol=1e-10
            end
            @test dot(f,B.efc_vel(rt.data))<=0
        finally
            C.contact_close!(rt)
        end
    end
end
end
