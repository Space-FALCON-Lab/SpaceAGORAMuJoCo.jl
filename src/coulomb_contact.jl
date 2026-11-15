# Coulomb force-level equations. These helpers do not commit contact modes or
# integrate a state. Accepted-step capture/projection belongs to the core driver.

"""Experimental force-level Coulomb contact with core-owned stick/slip events.
`capture_speed` is a numerical event tolerance, not a viscous creep parameter.
The initial hybrid implementation requires one isolated sphere/box-face,
capsule-end/box-face or sphere/sphere contact. Its full qualification is pending.
"""
struct CoulombContact <: CL.AbstractHybridContactModel
    geometry::PrimitiveContact
    capture_speed::Float64
    CoulombContact(geometry::PrimitiveContact, capture_speed::Real) = new(geometry,Float64(capture_speed))
end
function CoulombContact(geometries,pairs;capture_speed=1e-7,friction=.4,kwargs...)
    CoulombContact(PrimitiveContact(geometries,pairs;friction,kwargs...),Float64(capture_speed))
end
mutable struct CoulombContactRuntime <: CL.AbstractHybridContactRuntime
    primitive::PrimitiveContactRuntime
    config::CoulombContact
    sticking::Bool
    transitions::Vector{Tuple{Float64,Bool}}
end
function CL.contact_preflight(c::CoulombContact,bodies::NTuple{N,CL.ContactBody}) where {N}
    CL.contact_preflight(c.geometry,bodies)
    length(c.geometry.pairs)==1 || throw(ArgumentError("Initial Coulomb event coupling requires one explicit contact pair."))
    isfinite(c.capture_speed) && 0<c.capture_speed<=1e-5 && c.geometry.settings.friction>0 ||
        throw(ArgumentError("Coulomb coupling requires positive friction and a finite capture speed no larger than 1e-5 m/s."))
    nothing
end
function CL.contact_prepare(c::CoulombContact,bodies::NTuple{N,CL.ContactBody}) where {N}
    CL.contact_preflight(c,bodies)
    CoulombContactRuntime(CL.contact_prepare(c.geometry,bodies),c,false,Tuple{Float64,Bool}[])
end
CL.contact_close!(rt::CoulombContactRuntime)=CL.contact_close!(rt.primitive)

function _contact_rotation(q)
    x,y,z,w=q
    @SMatrix [1-2(y*y+z*z) 2(x*y-z*w) 2(x*z+y*w);
        2(x*y+z*w) 1-2(x*x+z*z) 2(y*z-x*w);
        2(x*z-y*w) 2(y*z+x*w) 1-2(x*x+y*y)]
end

function _contact_geometry_motion(rt,states,id)
    name=Binding.id2name(rt.model,5,id)
    g=rt.config.geometries[parse(Int,name[2:end])];s=states[g.body]
    R=_contact_rotation(s.quaternion);omega=R*s.angular_velocity_body_rad_s
    offset=R*g.position
    (;g,s,rotation=R*_contact_rotation(g.quaternion),center=s.position_m+offset,
        velocity=s.velocity_m_s+cross(omega,offset),omega)
end

# Differentiate the actual moving collision point and normal, rather than a
# material point frozen on either participant. Tangent axes are parallel-
# transported with the normal; their arbitrary in-plane spin is immaterial at
# zero tangential velocity.
function _contact_tangent_bias(rt,states,c)
    A=_contact_geometry_motion(rt,states,c.geom[1]);B=_contact_geometry_motion(rt,states,c.geom[2])
    frame=reshape(collect(c.frame),3,3);normal=SVector{3,Float64}(frame[:,1]);T=frame[:,2:3]
    point=SVector{3,Float64}(c.pos)
    if A.g.kind===:sphere && B.g.kind===:sphere
        separation=norm(B.center-A.center)
        relative=B.velocity-A.velocity
        normal_rate=(relative-normal*dot(normal,relative))/separation
        point_rate=(A.velocity+B.velocity)/2+(A.g.size[1]-B.g.size[1])/2*normal_rate
    elseif (A.g.kind===:box && B.g.kind in (:sphere,:capsule)) ||
           (B.g.kind===:box && A.g.kind in (:sphere,:capsule))
        box,round,sign=A.g.kind===:box ? (A,B,1.) : (B,A,-1.)
        outward=sign*normal
        local_normal=box.rotation'*outward
        maximum(abs,local_normal)>1-1e-10 || throw(ArgumentError("Exact sticking on box edges/corners is not yet qualified."))
        center=round.center;center_velocity=round.velocity
        if round.g.kind===:capsule
            axis=SVector{3,Float64}(round.rotation[:,3])
            endpoint=dot(point-center,axis)>=0 ? 1. : -1.
            offset=endpoint*round.g.size[2]*axis
            center+=offset;center_velocity+=cross(round.omega,offset)
        end
        outward_rate=cross(box.omega,outward)
        gap_rate=dot(outward_rate,center-box.center)+dot(outward,center_velocity-box.velocity)
        point_rate=center_velocity-gap_rate/2*outward-(round.g.size[1]+c.dist/2)*outward_rate
        normal_rate=sign*outward_rate
    else
        throw(ArgumentError("This primitive pair lacks a qualified sticking Jacobian derivative."))
    end
    relative=B.s.velocity_m_s+cross(B.omega,point-B.s.position_m)-
        A.s.velocity_m_s-cross(A.omega,point-A.s.position_m)
    acceleration=cross(B.omega,point_rate-B.s.velocity_m_s)-cross(A.omega,point_rate-A.s.velocity_m_s)
    T'*acceleration-(T'*normal_rate)*dot(normal,relative)
end

function _coulomb_query(rt::CoulombContactRuntime,states,t)
    p=rt.primitive;contacts=_primitive_terms!(p,states,t)
    active=filter(c->c.efc_address>=0,contacts)
    length(active)<=1 || throw(ArgumentError("Initial Coulomb event coupling requires one isolated contact point."))
    isempty(active) && return (;contacts,active=false,departure=false,f=Float64[],candidate=Float64[],margin=1.,velocity=zeros(2))
    c=only(active);J=Binding.constraint_jacobian(p.model,p.data);W=J*p.inverse_mass*J'
    Rn=Binding.efc_R(p.data)[1];a0=Binding.qacc_smooth(p.model,p.data)
    b=J*a0;b[1]-=Binding.efc_aref(p.data)[1]
    b[2:3].+=_contact_tangent_bias(p,states,c)
    velocity=copy(Binding.efc_vel(p.data)[2:3]);mu=p.config.settings.friction
    W[1,1]+Rn>mu*norm(W[1,2:3])+sqrt(eps(Float64))*opnorm(W) ||
        throw(ArgumentError("Coulomb response is outside the supported non-self-locking regime."))
    candidate=_coulomb_sticking_force(W,Rn,b)
    margin=min(candidate[1],mu*candidate[1]-norm(candidate[2:3]))
    # Resolve the direction at departure from zero slip through the Coulomb
    # acceleration equations. Directly normalizing roundoff-sized 2D velocity
    # otherwise produces orthogonal friction chatter after a cone-boundary
    # release. This numerical startup window is independently refined with
    # capture_speed; it introduces no viscous law or subcritical creep.
    departure=!rt.sticking && norm(velocity)<=rt.config.capture_speed && margin<=0
    departure_force=departure ? _coulomb_breakaway_force(W,Rn,b,mu;residual_limit=p.config.settings.residual_limit) : zeros(3)
    # Incoming slip must still be opposed before accepted capture. In
    # particular, roundoff at the startup-window boundary cannot flip its
    # friction power positive in a different inertial frame.
    outgoing=departure && dot(departure_force[2:3],velocity)<=0
    f=if rt.sticking
        candidate[1]>0 ? candidate : zeros(3)
    elseif outgoing
        departure_force
    elseif norm(velocity)>0
        _coulomb_sliding_force(W,Rn,b[1],velocity,mu)
    else
        _coulomb_breakaway_force(W,Rn,b,mu;residual_limit=p.config.settings.residual_limit)
    end
    # The sticking trial equations extend beyond the cone while the core locates
    # release. Only accepted modes may pass physical cone qualification.
    residual=abs(f[1]-max(0.,f[1]-((W*f+b)[1]+Rn*f[1])/(W[1,1]+Rn)))
    if rt.sticking && f[1]>0
        residual=max(residual,norm((W*f+b)[2:3])/minimum(eigvals(Symmetric(W[2:3,2:3]))))
    elseif outgoing
        acceleration=(W*f+b)[2:3]
        if norm(f[2:3])>0
            direction=-f[2:3]/norm(f[2:3])
            effective_min=minimum(eigvals(Symmetric(W[2:3,2:3])))
            residual=max(residual,abs(norm(f[2:3])-mu*f[1]),
                norm(acceleration-direction*max(0.,dot(direction,acceleration)))/effective_min)
        end
    elseif norm(velocity)>0
        residual=max(residual,norm(f[2:3]+mu*f[1]*velocity/norm(velocity)))
    end
    isfinite(residual) && residual<=p.config.settings.residual_limit || error("Unconverged Coulomb force certificate at $t s: $residual N (sticking=$(rt.sticking), departure=$departure).")
    (;contacts,active=true,departure,departure_force,f,candidate,margin,velocity,J=copy(J),W)
end

function CL.contact_evaluate!(rt::CoulombContactRuntime,states::NTuple{N,CL.ContactState},t::Float64) where {N}
    query=_coulomb_query(rt,states,t)
    _primitive_wrenches!(rt.primitive,states,query.contacts,query.f)
end
function CL.contact_event_value(rt::CoulombContactRuntime,states,t::Float64)
    q=_coulomb_query(rt,states,t)
    q.active || return 1.
    rt.sticking ? q.margin/(1+abs(q.candidate[1])) : norm(q.velocity)/rt.config.capture_speed-1
end
function CL.contact_accept!(rt::CoulombContactRuntime,states::NTuple{N,CL.ContactState},t::Float64) where {N}
    query=_coulomb_query(rt,states,t)
    old=rt.sticking;limit=rt.config.geometry.settings.residual_limit
    if !query.active || query.margin<=limit
        rt.sticking=false
    elseif !rt.sticking && query.margin>2limit && norm(query.velocity)<=rt.config.capture_speed*(1+1e-8)
        rt.sticking=true
    end
    old==rt.sticking || push!(rt.transitions,(t,rt.sticking))
    zero3=SVector(0.,0.,0.)
    if !rt.sticking && (!query.active || !query.departure || norm(query.departure_force[2:3])==0)
        return ntuple(_->CL.ContactVelocityCorrection(zero3,zero3),N)
    end
    rt.sticking && norm(query.velocity)>2rt.config.capture_speed && error("Sticking velocity drift exceeds the numerical capture budget.")
    Jt=query.J[2:3,:];inverse=rt.primitive.inverse_mass
    effective=Jt*inverse*Jt'
    target=zeros(2)
    if !rt.sticking
        direction=-query.departure_force[2:3]/norm(query.departure_force[2:3])
        speed=max(0.,dot(direction,effective\query.velocity)/dot(direction,effective\direction))
        target.=speed*direction
    end
    # Projection onto the outgoing velocity ray includes zero and minimizes
    # kinetic energy in the physical mass metric. It removes only numerical
    # transverse/reversed slip, and core records its dissipated energy.
    correction=inverse*Jt'*(effective\(target-query.velocity))
    norm(Jt*correction+query.velocity-target)<=1e-12 || error("Coulomb projection failed its velocity certificate.")
    ntuple(i->CL.ContactVelocityCorrection(SVector{3,Float64}(correction[6(i-1)+1:6(i-1)+3]),
        SVector{3,Float64}(correction[6(i-1)+4:6(i-1)+6])),N)
end

function _coulomb_sticking_force(W,Rn,b)
    size(W)==(3,3) && length(b)==3 || throw(DimensionMismatch("One normal and two tangent coordinates required."))
    H=copy(W);H[1,1]+=Rn
    minimum(eigvals(Symmetric(H)))>sqrt(eps(Float64))*opnorm(H) ||
        throw(ArgumentError("Sticking response is singular or ill-conditioned."))
    f=-(H\b)
    return f
end

function _coulomb_sliding_force(W,Rn,bn,velocity,mu)
    speed=norm(velocity)
    speed>0 || throw(ArgumentError("Sliding requires a nonzero slip direction."))
    direction=-mu*velocity/speed
    denominator=W[1,1]+Rn+dot(W[1,2:3],direction)
    denominator>sqrt(eps(Float64))*(W[1,1]+Rn+abs(dot(W[1,2:3],direction))) ||
        throw(ArgumentError("Coulomb normal response is outside the admissible regime."))
    fn=max(0.,-bn/denominator)
    vcat(fn,fn*direction)
end

# At release from zero slip, find an acceleration direction consistent with
# saturated Coulomb friction. Anisotropic tangential effective mass means this
# direction is not generally the smooth tangential acceleration direction.
function _coulomb_breakaway_force(W,Rn,b,mu;residual_limit=1e-10)
    candidate=_coulomb_sticking_force(W,Rn,b)
    candidate[1]>=0 && norm(candidate[2:3])<=mu*candidate[1] && return candidate
    b[1]>=0 && return zeros(3)
    # a_t = -lambda*f_t with lambda >= 0 fixes the outgoing acceleration
    # direction without normalizing a vanishing acceleration at breakaway.
    # The remaining scalar equation is ||f_t|| = mu*f_n. The full coupled
    # normal equation is included in every linear solve.
    H=copy(W);H[1,1]+=Rn
    force(lambda)=-(H+Diagonal([0.,lambda,lambda]))\b
    cone(f)=norm(f[2:3])-mu*f[1]
    lo=0.;hi=opnorm(H)
    for _ in 1:64
        cone(force(hi))<=0 && break
        hi*=2
    end
    cone(force(hi))<=0 || error("Coulomb breakaway could not bracket an admissible multiplier.")
    for _ in 1:80
        mid=(lo+hi)/2
        cone(force(mid))>0 ? (lo=mid) : (hi=mid)
    end
    lambda=(lo+hi)/2;f=force(lambda)
    residual=max(abs(cone(f)),abs((W*f+b)[1]+Rn*f[1])/(W[1,1]+Rn),
        norm((W*f+b)[2:3]+lambda*f[2:3])/minimum(eigvals(Symmetric(W[2:3,2:3]))))
    f[1]>=0 && isfinite(residual) && residual<=residual_limit ||
        error("Coulomb breakaway has no certified admissible acceleration direction.")
    f
end
