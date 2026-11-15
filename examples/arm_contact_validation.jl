using LinearAlgebra, StaticArrays, TOML

function arm_contact_observation(case,sc,t,rt,origin)
    C=SpaceAGORA.ContactLoadServices; SM=SpaceAGORA.SimulationModel
    CA=SM.ClothRobotArmDynamics;CM=SM.ClothMultibody;B=SpaceAGORAMuJoCo.Binding
    hybrid=rt isa SpaceAGORAMuJoCo.CoulombContactRuntime
    native=hybrid ? rt.primitive : rt
    settings=native.config.settings
    eff=only(case.args.dynamics_model.dynamic_effectors)
    F=zeros(3);T=zeros(3)
    smooth=CA._coupled_robot_arm_loads!(sc,case.plan,t,F,T;
        k_translation_n_m=eff.k_translation_n_m,c_translation_n_s_m=eff.c_translation_n_s_m,
        k_rotation_n_m_rad=eff.k_rotation_n_m_rad,c_rotation_n_m_s_rad=eff.c_rotation_n_m_s_rad)
    bodies=SpaceAGORA.SimulationEngine._contact_bodies(case.args)
    q=normalize(SVector{4,Float64}(sc.q));r=SVector{3,Float64}(sc.pos)-origin
    states=C.ContactState[C.ContactState(r,SVector{3,Float64}(sc.vel),q,SVector{3,Float64}(sc.ω),SVector{3,Float64}(F),CM._rot(q)*T)]
    for i in eachindex(case.plan.model.links)
        aq=normalize(SVector{4,Float64}(sc.arm_q[:,i]))
        push!(states,C.ContactState(r+SVector{3,Float64}(sc.arm_r[:,i]-sc.pos),SVector{3,Float64}(sc.arm_v[:,i]),aq,
            SVector{3,Float64}(sc.arm_ω[:,i]),smooth.forces[i],CM._rot(aq)*smooth.torques_body[i]))
    end
    states=Tuple(states);loads=C.contact_evaluate!(rt,states,Float64(t))
    zero3=SVector(0.,0.,0.);P=zero3;H=zero3;energy=0.;U=0.;damping=0.;max_angle=0.
    for (b,s) in zip(bodies,states)
        P+=b.mass_kg*s.velocity_m_s
        H+=cross(s.position_m,b.mass_kg*s.velocity_m_s)+CM._rot(s.quaternion)*(b.inertia_body_kg_m2*s.angular_velocity_body_rad_s)
        energy+=(b.mass_kg*dot(s.velocity_m_s,s.velocity_m_s)+dot(s.angular_velocity_body_rad_s,b.inertia_body_kg_m2*s.angular_velocity_body_rad_s))/2
    end
    rest=CA.cloth_robot_arm_rest_quaternions(case.plan,t)
    for (i,link) in enumerate(case.plan.model.links)
        parent_point=i==1 ? case.plan.model.mount_offset_body : case.plan.model.links[i-1].vector_parent-case.plan.model.links[i-1].com_offset_parent
        a=CA._coupled_parent_kinematics(sc,i-1,parent_point)
        b=CA._coupled_parent_kinematics(sc,i,-link.com_offset_parent)
        phi=CM._axis_angle_error(CM._quat_mul(CM._quat_mul(a.q,rest[i]),CM._quat_conj(b.q)))
        max_angle=max(max_angle,norm(phi))
        U+=(eff.k_translation_n_m*sum(abs2,a.point-b.point)+eff.k_rotation_n_m_rad*sum(abs2,phi))/2
        midpoint=(a.point+b.point)/2
        slip=a.point_velocity+cross(a.ω_world,midpoint-a.point)-b.point_velocity-cross(b.ω_world,midpoint-b.point)
        damping-=eff.c_translation_n_s_m*sum(abs2,slip)+eff.c_rotation_n_m_s_rad*sum(abs2,a.ω_world-b.ω_world)
    end
    power=sum(dot(w.force_n,s.velocity_m_s)+dot(w.torque_nm,CM._rot(s.quaternion)*s.angular_velocity_body_rad_s) for (w,s) in zip(loads,states))
    f=copy(B.efc_force(native.data));v=copy(B.efc_vel(native.data));dim=settings.friction==0 ? 1 : 3
    friction=sum((dot(f[k+1:k+dim-1],v[k+1:k+dim-1]) for k in 1:dim:length(f));init=0.)
    cone=maximum((f[k]>1e-9&&dim==3 ? norm(f[k+1:k+2])/(settings.friction*f[k]) : 0. for k in 1:dim:length(f));init=0.)
    slip=maximum((dim==3 ? norm(v[k+1:k+2]) : 0. for k in 1:dim:length(f));init=0.)
    gaps=[B.geom_distance(native.model,native.data,B.name2id(native.model,5,"g$a")+1,B.name2id(native.model,5,"g$b")+1) for (a,b) in native.config.pairs]
    (;time=Float64(t),states,loads,gap=minimum(gaps),contacts=B.ncon(native.data),P,H,energy=energy+U,
        cone,slip,sticking=hybrid&&rt.sticking,
        power,friction,damping,max_angle,power_projection=abs(power-dot(f,v)),
        pair_force=norm(sum(w.force_n for w in loads)),
        pair_moment=norm(sum(w.torque_nm+cross(s.position_m,w.force_n) for (w,s) in zip(loads,states))))
end

function arm_contact_ledger(case,sol;sample_step=2.0^-11)
    C=SpaceAGORA.ContactLoadServices
    rt=C.contact_prepare(case.contact,SpaceAGORA.SimulationEngine._contact_bodies(case.args))
    origin=SVector{3,Float64}(first(sol.u).sc[1].pos)
    hybrid=rt isa SpaceAGORAMuJoCo.CoulombContactRuntime
    run=sol.prob.p.shared_buffers.contact_runtime[]
    transitions=hybrid ? copy(run.runtime.transitions) : Tuple{Float64,Bool}[]
    function sample(t)
        if hybrid
            index=findlast(x->x[1]<=t,transitions)
            rt.sticking=index!==nothing && transitions[index][2]
        end
        arm_contact_observation(case,sol(t;continuity=:right).sc[1],t,rt,origin)
    end
    try
        times=sort!(unique!(vcat(collect(range(first(sol.t),last(sol.t);length=ceil(Int,(last(sol.t)-first(sol.t))/sample_step)+1)),first.(transitions))))
        rows=sample.(times)
        nodes=(-.8611363115940526,-.3399810435848563,.3399810435848563,.8611363115940526)
        weights=(.3478548451374538,.6521451548625461,.6521451548625461,.3478548451374538)
        function rawquad(a,b)
            m=(a+b)/2;h=(b-a)/2
            sum(map(nodes,weights) do x,w
                row=sample(m+h*x)
                h*w*SVector(row.power,row.damping,row.friction,row.loads[1].force_n...,row.sticking ? row.slip : 0.)
            end)
        end
        scales=SVector(1.3125,1.3125,1.3125,5.25,5.25,5.25,1.)
        function quad(a,b,depth=0)
            mid=(a+b)/2;coarse=rawquad(a,b);fine=rawquad(a,mid)+rawquad(mid,b)
            maximum(abs.((fine-coarse)./scales))<=max(1e-12,1e-9*(b-a)/(last(times)-first(times))) && return fine
            depth<30 || error("Arm contact quadrature failed to converge.")
            quad(a,mid,depth+1)+quad(mid,b,depth+1)
        end
        cumulative=[zero(SVector{7,Float64})]
        for i in 1:length(times)-1
            a,b=times[i],times[i+1]
            if rows[i].gap*rows[i+1].gap<0
                lo,hi=a,b
                for _ in 1:40
                    m=(lo+hi)/2
                    (sample(m).gap>0)==(rows[i].gap>0) ? (lo=m) : (hi=m)
                end
                mid=(lo+hi)/2
                push!(cumulative,last(cumulative)+quad(a,mid)+quad(mid,b))
            else
                push!(cumulative,last(cumulative)+quad(a,b))
            end
        end
        accepted=sample.(unique(sol.t))
        projection(t)=hasproperty(run,:projections) ? sum((x.work_J for x in run.projections if x.time_s<=t);init=0.) : (t==last(times) ? run.projection_work_J[] : 0.)
        metrics=Dict{String,Any}("duration_s"=>last(sol.t),"minimum_gap_m"=>minimum(r.gap for r in rows),
            "initial_gap_m"=>first(rows).gap,"final_gap_m"=>last(rows).gap,
            "maximum_contacts"=>maximum(r.contacts for r in rows),
            "momentum_residual_kg_m_s"=>maximum(norm(r.P-first(rows).P) for r in rows),
            "angular_momentum_residual_kg_m2_s"=>maximum(norm(r.H-first(rows).H) for r in rows),
            "energy_work_residual_J"=>maximum(abs(r.energy-first(rows).energy-w[1]-w[2]-projection(r.time)) for (r,w) in zip(rows,cumulative)),
            "mechanical_energy_change_J"=>last(rows).energy-first(rows).energy,
            "contact_work_J"=>last(cumulative)[1],"joint_damping_work_J"=>last(cumulative)[2],
            "friction_work_J"=>last(cumulative)[3],"maximum_friction_power_W"=>maximum(r.friction for r in accepted),
            "maximum_cone_ratio"=>maximum(r.cone for r in accepted),
            "maximum_sticking_speed_m_s"=>maximum((r.slip for r in accepted if r.sticking);init=0.),
            "sticking_slip_distance_m"=>last(cumulative)[7],
            "projection_work_J"=>run.projection_work_J[],"projection_count"=>run.projection_count[],
            "transitions"=>[Dict("time_s"=>t,"sticking"=>s) for (t,s) in transitions],
            "maximum_relative_joint_angle_rad"=>maximum(r.max_angle for r in rows),
            "force_balance_residual_N"=>maximum(r.pair_force for r in rows),
            "moment_balance_residual_Nm"=>maximum(r.pair_moment for r in rows),
            "power_projection_residual_W"=>maximum(r.power_projection for r in rows),
            "bus_contact_impulse_N_s"=>collect(last(cumulative)[4:6]))
        return (;rows,cumulative,accepted,metrics)
    finally
        C.contact_close!(rt)
    end
end
