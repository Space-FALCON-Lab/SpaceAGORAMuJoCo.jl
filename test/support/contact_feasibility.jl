# Bounded research harness, not a public adapter or a dynamics owner.
# Every query resets native data and supplies the complete trial state/loads.
struct ContactHarness
    model::Binding.MjModel
    data::Binding.MjData
    dim::Int
    mass_matrix::Diagonal{Float64,Vector{Float64}}
end

function ContactHarness(; dim=1, iterations=100, timestep=2.0^-12, mass=20.0)
    dim in (1,3) || throw(ArgumentError("feasibility fixture supports condim 1 or 3"))
    isfinite(mass) && mass > 0 || throw(ArgumentError("bus mass must be positive and finite"))
    inertia=mass .* [1/12, 1.36/12, 1.64/12]
    xml="""
    <mujoco>
      <compiler alignfree="false" inertiafromgeom="false"/>
      <option timestep="$timestep" gravity="0 0 0" solver="Newton" jacobian="dense"
              cone="elliptic" tolerance="1e-12" iterations="$iterations">
        <flag warmstart="disable" sleep="disable" diagexact="enable"/>
      </option>
      <worldbody>
        <body name="bus"><freejoint align="false"/>
          <inertial pos="0 0 0" mass="$mass" diaginertia="$(join(inertia,' '))"/>
          <geom name="stop" type="box" size="0.5 0.4 0.3" mass="0" contype="0" conaffinity="0"/>
        </body>
        <body name="bumper" pos="0.61 0 0"><freejoint align="false"/>
          <inertial pos="0 0 0" mass="1" diaginertia="0.004 0.004 0.004"/>
          <geom name="ball" type="sphere" size="0.1" mass="0" contype="0" conaffinity="0"/>
        </body>
      </worldbody>
      <contact><pair geom1="stop" geom2="ball" condim="$dim" margin="0" gap="0"
        friction="0.4 0.4 0 0 0" solref="0.01 1" solimp="0.9 0.9 0.001 0.5 2"/></contact>
    </mujoco>
    """
    m=Binding.load_xml_string(xml)
    try
        return ContactHarness(m,Binding.MjData(m),dim,Diagonal(vcat(fill(mass,3),inertia,ones(3),fill(.004,3))))
    catch
        close(m)
        rethrow()
    end
end

Base.close(h::ContactHarness) = (close(h.data); close(h.model); nothing)

# Explicit quaternion rotation, independent of MuJoCo's matrix readback.
function probe_rotation(q)
    w,x,y,z=q
    return [1-2(y*y+z*z) 2(x*y-z*w) 2(x*z+y*w);
            2(x*y+z*w) 1-2(x*x+z*z) 2(y*z-x*w);
            2(x*z-y*w) 2(y*z+x*w) 1-2(x*x+y*y)]
end

function project_soc(z)
    r=norm(z[2:end]); a=z[1]
    r <= a && return copy(z)
    r <= -a && return zeros(length(z))
    return vcat((a+r)/2, ((a+r)/(2r)) .* z[2:end])
end

# A force-valued projected-gradient KKT residual of the convex contact problem.
# M is independently constructed from physical masses/inertias, not mjData.
function contact_residual(h,J,R,aref,a0,f)
    isempty(f) && return 0.0
    Q=J*(h.mass_matrix\J')+Diagonal(R)
    b=J*a0-aref
    if h.dim == 1
        return norm(f-max.(0.0,f-(Q*f+b)/opnorm(Q)))
    end
    # f = D*z, z in the unit second-order cone for isotropic sliding friction.
    D=Diagonal([1.0,.4,.4]); z=D\f
    Qz=D*Q*D; bz=D*b
    return norm(D*(z-project_soc(z-(Qz*z+bz)/opnorm(Qz))))
end

function contact_query(h::ContactHarness,q,v,wrenches; time=0.0, residual_limit=1e-10)
    length(q)==14 && length(v)==12 && size(wrenches)==(6,2) ||
        throw(DimensionMismatch("expected two free bodies and two COM wrenches"))
    all(isfinite,q) && all(isfinite,v) && all(isfinite,wrenches) && isfinite(time) ||
        throw(ArgumentError("trial state and loads must be finite"))
    all(abs(norm(q[k:k+3])-1)<1e-12 for k in (4,11)) ||
        throw(ArgumentError("trial quaternions must be unit length"))
    m,d=h.model,h.data
    Binding.reset_data!(m,d)
    Binding.qpos(m,d) .= q
    Binding.qvel(m,d) .= v
    Binding.set_data_time!(d,Float64(time))
    Binding.xfrc_applied(m,d)[:,2:3] .= wrenches
    Binding.forward!(m,d)
    all(Binding.warning_count(d,i)==0 for i in 0:6) || error("native contact query raised a warning")
    Binding.ncon(d) <= 1 || error("fixture only supports one contact")
    Binding.nefc(d) in (0,h.dim) || error("unexpected native constraints")
    J=copy(Binding.constraint_jacobian(m,d)); R=copy(Binding.efc_R(d))
    a0=copy(Binding.qacc_smooth(m,d)); f=copy(Binding.efc_force(d))
    aref=copy(Binding.efc_aref(d)); acc=copy(Binding.qacc(m,d))
    all(isfinite,acc) && all(isfinite,f) || error("nonfinite native contact result")
    residual=contact_residual(h,J,R,aref,a0,f)
    isfinite(residual) && residual <= residual_limit || error("unconverged contact solve: residual=$residual N")
    loads=zeros(6,2); contacts=NamedTuple[]
    for k in 1:Binding.ncon(d)
        c=Binding.contact(d,k)
        cf=Binding.contact_force!(zeros(6),m,d,k)
        basis=reshape(collect(c.frame),3,3)
        force=basis*cf[1:3]; couple=basis*cf[4:6]
        for (geom,sign) in zip(c.geom,(-1,1))
            body=Binding.geom_bodyid(m)[geom+1]
            body in (1,2) || error("unexpected contact participant")
            r=q[7(body-1)+1:7(body-1)+3]
            loads[1:3,body] .+= sign.*force
            loads[4:6,body] .+= sign.*(cross(collect(c.pos)-r,force)+couple)
        end
        push!(contacts,(; c..., force=cf))
    end
    return (; loads,contacts,J,R,aref,a0,f,acc,residual,
        generalized=copy(Binding.qfrc_constraint(m,d)),
        velocity=copy(Binding.efc_vel(d)),position=copy(Binding.efc_pos(d)),
        diagA=copy(Binding.efc_diagA(d)))
end

probe_state(; x=.599,y=0.0) = ([0.,0.,0.,1.,0.,0.,0.,x,y,0.,1.,0.,0.,0.],zeros(12),zeros(6,2))
