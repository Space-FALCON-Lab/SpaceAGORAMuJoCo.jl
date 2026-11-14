# Experimental state-to-wrench provider. Integration remains entirely in core.
CL.require_version(v"0.1.0")

"""
    BusBumperContact(; bus_half_size=(0.5,0.4,0.3), bumper_radius=0.1, friction=0.0,
        timestep=2.0^-12, time_constant=0.01, damping_ratio=1.0, impedance=0.9,
        iterations=100, tolerance=1e-12, residual_limit=1e-10)

Experimental contact between a box centered on the rigid bus COM and a sphere
centered on its sole compliant attachment body. Core supplies both physical
inertias and every trial state/smooth COM wrench. Native gravity and other force
generators are disabled. The native timestep is fixed contact regularization,
not an integration step. Requires ContactLoadServices v0.1.0; install into a
configuration with its `with_contact_model` helper. No native state is integrated.
"""
Base.@kwdef struct BusBumperContact <: CL.AbstractContactModel
    bus_half_size::SVector{3,Float64} = SVector(.5,.4,.3)
    bumper_radius::Float64 = .1
    friction::Float64 = 0.0
    timestep::Float64 = 2.0^-12
    time_constant::Float64 = .01
    damping_ratio::Float64 = 1.0
    impedance::Float64 = .9
    iterations::Int = 100
    tolerance::Float64 = 1e-12
    residual_limit::Float64 = 1e-10
end

struct BusBumperRuntime <: CL.AbstractContactRuntime
    model::Binding.MjModel
    data::Binding.MjData
    config::BusBumperContact
    inverse_mass::Matrix{Float64}
end

function CL.contact_preflight(c::BusBumperContact, bodies::NTuple{2,CL.ContactBody})
    CL.require_version(v"0.1.0")
    all(x -> isfinite(x) && x > 0, c.bus_half_size) &&
        all(x -> isfinite(x) && x > 0, (c.bumper_radius,c.timestep,c.time_constant,c.damping_ratio,c.tolerance,c.residual_limit)) ||
        throw(ArgumentError("Contact geometry and numerical settings must be positive and finite."))
    isfinite(c.friction) && c.friction >= 0 && 0 < c.impedance < 1 && c.iterations > 0 ||
        throw(ArgumentError("Contact requires finite nonnegative friction, 0 < impedance < 1 and a positive iteration cap."))
    c.time_constant >= 2c.timestep || throw(ArgumentError("Contact time constant must be >= twice native timestep (refsafe must not clamp it)."))
    c.residual_limit <= 1e-10 || throw(ArgumentError("Contact residual limit must be <= 1e-10 N."))
    return nothing
end

function CL.contact_prepare(c::BusBumperContact, bodies::NTuple{2,CL.ContactBody})
    CL.contact_preflight(c,bodies)
    inertials=map(bodies) do b
        # Supply a high-accuracy principal frame explicitly: MuJoCo's XML
        # fullinertia eigensolver otherwise leaves ~1e-9 frame errors.
        e=eigen(Symmetric(Matrix(b.inertia_body_kg_m2)))
        R=copy(e.vectors)
        det(R)<0 && (R[:,3] .*= -1)
        K=[R[1,1]-R[2,2]-R[3,3] R[1,2]+R[2,1] R[1,3]+R[3,1] R[3,2]-R[2,3];
           R[1,2]+R[2,1] R[2,2]-R[1,1]-R[3,3] R[2,3]+R[3,2] R[1,3]-R[3,1];
           R[1,3]+R[3,1] R[2,3]+R[3,2] R[3,3]-R[1,1]-R[2,2] R[2,1]-R[1,2];
           R[3,2]-R[2,3] R[1,3]-R[3,1] R[2,1]-R[1,2] tr(R)]
        q=eigen(Symmetric(K)).vectors[:,end]
        "<inertial pos=\"0 0 0\" mass=\"$(b.mass_kg)\" diaginertia=\"$(join(e.values,' '))\" quat=\"$(join(q[[4,1,2,3]],' '))\"/>"
    end
    dim = c.friction == 0 ? 1 : 3
    xml="""
    <mujoco><compiler alignfree="false" inertiafromgeom="false"/>
      <option gravity="0 0 0" timestep="$(c.timestep)" jacobian="dense" cone="elliptic"
        solver="Newton" iterations="$(c.iterations)" tolerance="$(c.tolerance)">
        <flag warmstart="disable" sleep="disable" diagexact="enable"/>
      </option><worldbody>
        <body name="bus"><freejoint align="false"/>$(inertials[1])
          <geom name="bus_box" type="box" size="$(join(c.bus_half_size,' '))" mass="0" contype="0" conaffinity="0"/>
        </body>
        <body name="bumper"><freejoint align="false"/>$(inertials[2])
          <geom name="bumper_sphere" type="sphere" size="$(c.bumper_radius)" mass="0" contype="0" conaffinity="0"/>
        </body>
      </worldbody><contact>
        <pair geom1="bus_box" geom2="bumper_sphere" condim="$dim" margin="0" gap="0"
          friction="$(c.friction) $(c.friction) 0 0 0" solref="$(c.time_constant) $(c.damping_ratio)"
          solimp="$(c.impedance) $(c.impedance) .001 .5 2"/>
      </contact></mujoco>
    """
    m=Binding.load_xml_string(xml)
    try
        inverse=zeros(12,12)
        for i in 1:2
            k=6(i-1)
            inverse[k+1:k+3,k+1:k+3] .= Matrix{Float64}(I,3,3)/bodies[i].mass_kg
            inverse[k+4:k+6,k+4:k+6] .= inv(bodies[i].inertia_body_kg_m2)
        end
        return BusBumperRuntime(m,Binding.MjData(m),c,inverse)
    catch
        close(m)
        rethrow()
    end
end

function CL.contact_close!(rt::BusBumperRuntime)
    close(rt.data)
    close(rt.model)
    return nothing
end

function _contact_cone_projection(z)
    r=norm(z[2:3]); a=z[1]
    r<=a && return z
    r<=-a && return zeros(3)
    return vcat((a+r)/2, (a+r)/(2r).*z[2:3])
end

function _contact_optimality_residual(rt,J,R,aref,a0,f)
    isempty(f) && return 0.0
    A=J*rt.inverse_mass*J'+Diagonal(R)
    b=J*a0-aref
    if rt.config.friction==0
        return norm(f-max.(0.,f-(A*f+b)/opnorm(A)))
    end
    D=Diagonal([1.,rt.config.friction,rt.config.friction])
    z=D\f; H=D*A*D
    return norm(D*(z-_contact_cone_projection(z-(H*z+D*b)/opnorm(H))))
end

function CL.contact_evaluate!(rt::BusBumperRuntime, states::NTuple{2,CL.ContactState}, t::Float64)
    all(s -> all(name -> all(isfinite,getfield(s,name)),fieldnames(CL.ContactState)),states) && isfinite(t) ||
        throw(ArgumentError("Contact query requires finite states, loads and time."))
    all(s -> abs(norm(s.quaternion)-1)<1e-12,states) || throw(ArgumentError("Contact quaternions must have unit length."))
    m,d=rt.model,rt.data
    Binding.reset_data!(m,d)
    q=Binding.qpos(m,d); v=Binding.qvel(m,d); w=Binding.xfrc_applied(m,d)
    for i in 1:2
        state=states[i]; k=7(i-1); j=6(i-1)
        q[k+1:k+3] .= state.position_m
        q[k+4:k+7] .= state.quaternion[[4,1,2,3]]
        v[j+1:j+3] .= state.velocity_m_s
        v[j+4:j+6] .= state.angular_velocity_body_rad_s
        w[1:3,i+1] .= state.smooth_force_n
        w[4:6,i+1] .= state.smooth_torque_nm
    end
    Binding.set_data_time!(d,t)
    Binding.forward!(m,d)
    all(Binding.warning_count(d,i)==0 for i in 0:6) || error("Native contact query raised a warning or exhausted capacity.")
    dim=rt.config.friction==0 ? 1 : 3
    Binding.ncon(d)<=1 && Binding.nefc(d) in (0,dim) || error("Unexpected native constraints in the bus–bumper fixture.")
    f=Binding.efc_force(d)
    all(isfinite,Binding.qacc(m,d)) && all(isfinite,f) || error("Nonfinite native contact result.")
    residual=_contact_optimality_residual(rt,Binding.constraint_jacobian(m,d),Binding.efc_R(d),
        Binding.efc_aref(d),Binding.qacc_smooth(m,d),f)
    isfinite(residual) && residual<=rt.config.residual_limit || error("Unconverged contact solve: residual=$residual N.")
    forces=zeros(3,2); torques=zeros(3,2)
    for index in 1:Binding.ncon(d)
        contact=Binding.contact(d,index)
        load=Binding.contact_force!(zeros(6),m,d,index)
        frame=reshape(collect(contact.frame),3,3)
        force=frame*load[1:3]; couple=frame*load[4:6]
        for (geom,sign) in zip(contact.geom,(-1,1))
            body=Binding.geom_bodyid(m)[geom+1]
            body in (1,2) || error("Unexpected contact body.")
            forces[:,body] .+= sign.*force
            torques[:,body] .+= sign.*(cross(SVector{3,Float64}(contact.pos)-states[body].position_m,force)+couple)
        end
    end
    all(isfinite,forces) && all(isfinite,torques) || error("Nonfinite contact wrench.")
    return ntuple(i -> CL.ContactWrench(SVector{3,Float64}(forces[:,i]),SVector{3,Float64}(torques[:,i])),2)
end
