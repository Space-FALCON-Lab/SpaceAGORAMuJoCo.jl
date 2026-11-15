module CoulombGeometryTests
using Test, LinearAlgebra, StaticArrays
using ..SpaceAGORAMuJoCo
const M=SpaceAGORAMuJoCo;const C=M.CL;const B=M.Binding
const Z=SVector(0.,0.,0.);const Q=SVector(0.,0.,0.,1.)
state(r;v=Z,q=Q,w=Z)=C.ContactState(SVector{3,Float64}(r),v,q,w,Z,Z)
function advance(s,h)
    speed=norm(s.angular_velocity_body_rad_s)
    d=speed==0 ? Q : SVector((sin(h*speed/2)/speed*s.angular_velocity_body_rad_s)...,cos(h*speed/2))
    a=s.quaternion[1:3];b=d[1:3]
    q=SVector((s.quaternion[4]*b+d[4]*a+cross(a,b))...,s.quaternion[4]*d[4]-dot(a,b))
    state(s.position_m+h*s.velocity_m_s;v=s.velocity_m_s,q=q,w=s.angular_velocity_body_rad_s)
end
function observation(rt,states)
    c=only(M._primitive_terms!(rt,states,0.))
    p=SVector{3,Float64}(c.pos);frame=reshape(collect(c.frame),3,3)
    ga=M._contact_geometry_motion(rt,states,c.geom[1]);gb=M._contact_geometry_motion(rt,states,c.geom[2])
    relative=gb.s.velocity_m_s+cross(gb.omega,p-gb.s.position_m)-ga.s.velocity_m_s-cross(ga.omega,p-ga.s.position_m)
    (;c,normal=frame[:,1],T=frame[:,2:3],relative)
end
@testset "moving collision Jacobian derivative" begin
    bodies=(C.ContactBody(20.,[2. .1 .2;.1 3. .1;.2 .1 4.]),C.ContactBody(1.,Diagonal([.02,.03,.04])))
    for kind in (:sphere,:capsule,:spheres), scale in (0.,.3,1.), reversed in (false,true)
        a=kind==:spheres ? ContactGeometry(1,:sphere,(.5,)) : ContactGeometry(1,:box,(.5,.4,.3))
        b=kind==:capsule ? ContactGeometry(2,:capsule,(.04,.1);quaternion=(0.,sin(pi/4),0.,cos(pi/4))) : ContactGeometry(2,:sphere,(.1,))
        geoms=reversed ? (b,a) : (a,b)
        rt=C.contact_prepare(PrimitiveContact(geoms,((1,2),)),bodies)
        try
            states=(state(Z;v=scale*SVector(.1,-.3,.2),w=scale*SVector(.2,.3,-.4)),
                state((kind==:capsule ? .639 : .599,kind==:spheres ? 0. : .04,0.);v=scale*SVector(-.2,.5,.3),w=scale*SVector(-.4,.2,.1)))
            base=observation(rt,states);expected=M._contact_tangent_bias(rt,states,base.c)
            for h in (1e-5,1e-6)
                plus=observation(rt,map(s->advance(s,h),states));minus=observation(rt,map(s->advance(s,-h),states))
                derivative=base.T'*(plus.relative-minus.relative)/(2h)-
                    (base.T'*(plus.normal-minus.normal)/(2h))*dot(base.normal,base.relative)
                @test norm(expected-derivative)<1e-7
            end
        finally
            C.contact_close!(rt)
        end
    end
end
end
