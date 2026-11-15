"""Collision-only primitive on a physical body. `body` is its one-based contract
slot; sizes are radius (sphere), half extents (box), or radius and half cylindrical
length (capsule). Pose is body-local, with a scalar-last quaternion. Geometries
never contribute a second mass or inertia.
"""
struct ContactGeometry
    body::Int
    kind::Symbol
    size::Tuple{Vararg{Float64}}
    position::SVector{3,Float64}
    quaternion::SVector{4,Float64}
end
function ContactGeometry(body,kind,size;position=(0.,0.,0.),quaternion=(0.,0.,0.,1.))
    ContactGeometry(Int(body),Symbol(kind),Tuple(Float64.(size)),SVector{3,Float64}(position),SVector{4,Float64}(quaternion))
end

"""Explicit primitive pairs for a rigid bus and compliant links. Core owns every
body's dynamics. Friction is passive regularized Coulomb friction; this provider
does not claim exact sticking. The supported primitive set is sphere/box/capsule.
"""
struct PrimitiveContact <: CL.AbstractContactModel
    geometries::Tuple{Vararg{ContactGeometry}}
    pairs::Tuple{Vararg{Tuple{Int,Int}}}
    settings::BusBumperContact
end
function PrimitiveContact(geometries,pairs;kwargs...)
    PrimitiveContact(Tuple(geometries),Tuple((Int(a),Int(b)) for (a,b) in pairs),BusBumperContact(;kwargs...))
end

struct PrimitiveContactRuntime <: CL.AbstractContactRuntime
    model::Binding.MjModel
    data::Binding.MjData
    config::PrimitiveContact
    inverse_mass::Matrix{Float64}
    bodies::Int
end

function CL.contact_preflight(c::PrimitiveContact,bodies::NTuple{N,CL.ContactBody}) where {N}
    N>=2 || throw(ArgumentError("Primitive contact requires at least two physical bodies."))
    CL.contact_preflight(c.settings,(bodies[1],bodies[2]))
    !isempty(c.geometries) && !isempty(c.pairs) || throw(ArgumentError("Explicit geometries and collision pairs are required."))
    for g in c.geometries
        1<=g.body<=N || throw(ArgumentError("Geometry refers to a missing physical body."))
        dimensions=g.kind===:sphere ? 1 : g.kind===:box ? 3 : g.kind===:capsule ? 2 : 0
        dimensions>0 && length(g.size)==dimensions && all(x->isfinite(x)&&x>0,g.size) ||
            throw(ArgumentError("Geometry requires positive finite sphere/box/capsule dimensions."))
        all(isfinite,g.position) && all(isfinite,g.quaternion) && abs(norm(g.quaternion)-1)<1e-12 ||
            throw(ArgumentError("Geometry pose must be finite with a unit quaternion."))
    end
    canonical=Set{Tuple{Int,Int}}()
    for (a,b) in c.pairs
        1<=a<=length(c.geometries) && 1<=b<=length(c.geometries) && a!=b ||
            throw(ArgumentError("Collision pair indices must name two distinct geometries."))
        c.geometries[a].body!=c.geometries[b].body || throw(ArgumentError("A contact pair must join distinct physical bodies."))
        pair=minmax(a,b)
        pair in canonical && throw(ArgumentError("Duplicate collision pair."))
        push!(canonical,pair)
    end
    nothing
end

function CL.contact_prepare(c::PrimitiveContact,bodies::NTuple{N,CL.ContactBody}) where {N}
    CL.contact_preflight(c,bodies)
    s=c.settings; dim=s.friction==0 ? 1 : 3
    body_xml=map(enumerate(bodies)) do (i,b)
        geoms=String[]
        for (j,g) in enumerate(c.geometries)
            g.body==i || continue
            push!(geoms,"<geom name=\"g$j\" type=\"$(g.kind)\" size=\"$(join(g.size,' '))\" pos=\"$(join(g.position,' '))\" quat=\"$(join(g.quaternion[[4,1,2,3]],' '))\" mass=\"0\" contype=\"0\" conaffinity=\"0\"/>")
        end
        "<body name=\"b$i\"><freejoint align=\"false\"/>$(_contact_inertial_xml(b))$(join(geoms))</body>"
    end
    pairs=["<pair geom1=\"g$a\" geom2=\"g$b\" condim=\"$dim\" margin=\"0\" gap=\"0\" friction=\"$(s.friction) $(s.friction) 0 0 0\" solref=\"$(s.time_constant) $(s.damping_ratio)\" solimp=\"$(s.impedance) $(s.impedance) .001 .5 2\"/>" for (a,b) in c.pairs]
    xml="""<mujoco><compiler alignfree="false" inertiafromgeom="false"/>
    <option gravity="0 0 0" timestep="$(s.timestep)" jacobian="dense" cone="elliptic">
    <flag warmstart="disable" sleep="disable" diagexact="enable"/></option>
    <worldbody>$(join(body_xml))</worldbody><contact>$(join(pairs))</contact></mujoco>"""
    m=Binding.load_xml_string(xml)
    try
        inverse=zeros(6N,6N)
        for (i,b) in enumerate(bodies)
            k=6(i-1)
            inverse[k+1:k+3,k+1:k+3].=Matrix{Float64}(I,3,3)/b.mass_kg
            inverse[k+4:k+6,k+4:k+6].=inv(b.inertia_body_kg_m2)
        end
        PrimitiveContactRuntime(m,Binding.MjData(m),c,inverse,N)
    catch
        close(m);rethrow()
    end
end
function CL.contact_close!(rt::PrimitiveContactRuntime)
    close(rt.data);close(rt.model);nothing
end

# With velocity fixed, passive friction directions are known. Substituting them
# leaves one normal complementarity equation per contact. A positive-definite
# symmetric part ensures a unique monotone LCP. Enumerate active sets for this
# bounded dense solver and certify the original full contact equations below.
function _primitive_regularized_forces(W,R,b,velocity,dim,mu,vs,limit)
    count=length(b)÷dim
    count<=8 || throw(ArgumentError("The qualified dense contact solver supports at most eight simultaneous contacts."))
    count==0 && return Float64[]
    normal=collect(1:dim:length(b)); C=zeros(length(b),count)
    for i in 1:count
        k=normal[i]; C[k,i]=1.
        vt=velocity[k+1:k+dim-1]
        C[k+1:k+dim-1,i].=-mu.*vt/max(vs,norm(vt))
    end
    A=W[normal,:]*C+Diagonal(R[normal])
    minimum(eigvals(Symmetric((A+A')/2)))>sqrt(eps(Float64))*opnorm(A) ||
        throw(ArgumentError("Multiple-contact normal response is outside the supported monotone regime."))
    for mask in 0:(1<<count)-1
        active=[i for i in 1:count if (mask>>(i-1))&1==1]
        fn=zeros(count)
        isempty(active) || (fn[active].=-(A[active,active]\b[normal[active]]))
        all(>=(0.),fn) || continue
        f=C*fn; g=W*f+b
        residual=0.
        for (i,k) in enumerate(normal)
            gn=g[k]+R[k]*fn[i]
            residual=max(residual,abs(fn[i]-max(0.,fn[i]-gn/(W[k,k]+R[k]))))
            vt=velocity[k+1:k+dim-1];ft=f[k+1:k+dim-1]
            residual=max(residual,norm(ft+mu*fn[i].*vt/max(vs,norm(vt))),norm(ft)-mu*fn[i])
        end
        isfinite(residual) && residual<=limit && return f
    end
    error("Unconverged multiple-contact solve: no active set meets the force certificate.")
end

function _primitive_terms!(rt::PrimitiveContactRuntime,states::NTuple{N,CL.ContactState},t::Float64) where {N}
    N==rt.bodies || throw(ArgumentError("Contact query body count changed during a run."))
    isfinite(t) && all(s->all(name->all(isfinite,getfield(s,name)),fieldnames(CL.ContactState)),states) &&
        all(s->abs(norm(s.quaternion)-1)<1e-12,states) || throw(ArgumentError("Invalid primitive-contact state."))
    m,d=rt.model,rt.data;s=rt.config.settings
    Binding.reset_data!(m,d)
    q=Binding.qpos(m,d);v=Binding.qvel(m,d);w=Binding.xfrc_applied(m,d)
    for (i,state) in enumerate(states)
        k=7(i-1);j=6(i-1)
        q[k+1:k+3].=state.position_m;q[k+4:k+7].=state.quaternion[[4,1,2,3]]
        v[j+1:j+3].=state.velocity_m_s;v[j+4:j+6].=state.angular_velocity_body_rad_s
        w[1:3,i+1].=state.smooth_force_n;w[4:6,i+1].=state.smooth_torque_nm
    end
    Binding.set_data_time!(d,t);Binding.contact_terms!(m,d)
    all(Binding.warning_count(d,i)==0 for i in 0:6) || error("Native primitive-contact query raised a warning.")
    dim=s.friction==0 ? 1 : 3
    contacts=[Binding.contact(d,i) for i in 1:Binding.ncon(d)]
    active=filter(c->c.efc_address>=0,contacts)
    length(active)*dim==Binding.nefc(d) && all(c->c.dim==dim,active) || error("Unexpected native constraints.")
    sort([c.efc_address for c in active])==collect(0:dim:Binding.nefc(d)-1) || error("Unexpected native contact row layout.")
    return contacts
end

function _primitive_wrenches!(rt::PrimitiveContactRuntime,states::NTuple{N,CL.ContactState},contacts,f) where {N}
    m,d=rt.model,rt.data
    J=Binding.constraint_jacobian(m,d);a0=Binding.qacc_smooth(m,d)
    Binding.efc_force(d).=f
    Binding.qfrc_constraint(m,d).=J'*f
    Binding.qacc(m,d).=a0+rt.inverse_mass*Binding.qfrc_constraint(m,d)
    all(isfinite,f) && all(isfinite,Binding.qacc(m,d)) || error("Nonfinite primitive-contact solve.")
    F=zeros(3,N);T=zeros(3,N)
    for (index,c) in enumerate(contacts)
        load=Binding.contact_force!(zeros(6),m,d,index);frame=reshape(collect(c.frame),3,3)
        force=frame*load[1:3];couple=frame*load[4:6]
        for (geom,sign) in zip(c.geom,(-1,1))
            body=Binding.geom_bodyid(m)[geom+1]
            1<=body<=N || error("Unexpected native contact participant.")
            F[:,body].+=sign.*force
            T[:,body].+=sign.*(cross(SVector{3,Float64}(c.pos)-states[body].position_m,force)+couple)
        end
    end
    all(isfinite,F) && all(isfinite,T) || error("Nonfinite primitive-contact wrench.")
    ntuple(i->CL.ContactWrench(SVector{3,Float64}(F[:,i]),SVector{3,Float64}(T[:,i])),N)
end

function CL.contact_evaluate!(rt::PrimitiveContactRuntime,states::NTuple{N,CL.ContactState},t::Float64) where {N}
    contacts=_primitive_terms!(rt,states,t)
    m,d=rt.model,rt.data;s=rt.config.settings;dim=s.friction==0 ? 1 : 3
    J=Binding.constraint_jacobian(m,d);a0=Binding.qacc_smooth(m,d)
    f=_primitive_regularized_forces(J*rt.inverse_mass*J',Binding.efc_R(d),J*a0-Binding.efc_aref(d),
        Binding.efc_vel(d),dim,s.friction,s.slip_speed,s.residual_limit)
    _primitive_wrenches!(rt,states,contacts,f)
end
