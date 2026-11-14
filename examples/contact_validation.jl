# Analysis of the supported two-body fixture through public core APIs.
# No native time step and no private core engine calls are used here.
using SpaceAGORA, LinearAlgebra, StaticArrays

contact_rotation(q) = begin
    x,y,z,w=normalize(SVector{4,Float64}(q))
    @SMatrix [1-2(y*y+z*z) 2(x*y-z*w) 2(x*z+y*w);
        2(x*y+z*w) 1-2(x*x+z*z) 2(y*z-x*w);
        2(x*z-y*w) 2(y*z+x*w) 1-2(x*x+y*y)]
end
function contact_quaternion_product(a,b)
    av=SVector(a[1],a[2],a[3]); bv=SVector(b[1],b[2],b[3])
    v=a[4]*bv+b[4]*av+cross(av,bv)
    return SVector(v[1],v[2],v[3],a[4]*b[4]-dot(av,bv))
end
contact_angle(a,b) = 2acos(clamp(abs(dot(normalize(a),normalize(b))),0.,1.))

function contact_analysis_model(args)
    sc=only(args.dynamics_model.spacecraft); attachment=only(sc.attachments)
    joint=only(attachment.model.joints)
    # The fixture has identity mount attitude. Turn its mount-to-child joint
    # into a bus-to-child joint for the public standalone diagnostic function.
    attachment.mount_quaternion==SVector(0.,0.,0.,1.) || error("Analysis requires identity mount attitude")
    bodies=[CompliantBody(:bus,sc.dry_mass+sc.prop_mass,sc.inertia_tensor),only(attachment.model.bodies)]
    j=CompliantJoint(joint.name,1,2,attachment.mount_point+joint.parent_point_body,joint.child_point_body,
        joint.k_translation_n_m,joint.c_translation_n_s_m,joint.k_rotation_n_m_rad,
        joint.c_rotation_n_m_s_rad,joint.rest_child_parent_quat)
    model=CompliantMultibodyModel(bodies,[j],SVector(0.,0.,0.),SVector(0.,0.,0.,1.))
    return model
end

function contact_observation(model,sv,t,origin,frame_velocity,rt)
    C=SpaceAGORA.ContactLoadServices; B=SpaceAGORAMuJoCo.Binding
    j=only(model.joints)
    r=SVector{3,Float64}(sv.pos)-origin-t*frame_velocity
    a=SVector{3,Float64}(sv.att_r[:,1]); v=SVector{3,Float64}(sv.vel)-frame_velocity
    av=v+SVector{3,Float64}(sv.att_v[:,1])
    q=normalize(SVector{4,Float64}(sv.q)); aq=normalize(SVector{4,Float64}(sv.att_q[:,1]))
    w=SVector{3,Float64}(sv.ω); aw=SVector{3,Float64}(sv.att_ω[:,1])
    R,AR=contact_rotation(q),contact_rotation(aq)
    x=compliant_state_vector([r,r+a],[q,aq];velocities=[v,av],angular_velocities=[w,aw])
    jointload=only(compliant_joint_loads(model,x))
    f=(jointload.translation_force_parent_world,jointload.translation_force_child_world)
    tau=(cross(R*j.parent_point_body,f[1])+jointload.compliance_torque_parent_world,
        cross(AR*j.child_point_body,f[2])+jointload.compliance_torque_child_world)
    states=(C.ContactState(r,v,q,w,f[1],tau[1]),C.ContactState(r+a,av,aq,aw,f[2],tau[2]))
    loads=C.contact_evaluate!(rt,states,Float64(t))
    contact_power=sum(dot(loads[i].force_n,states[i].velocity_m_s)+
        dot(loads[i].torque_nm,contact_rotation(states[i].quaternion)*states[i].angular_velocity_body_rad_s) for i in 1:2)
    # Analytic potential and damping power, independent of the joint-load kernel.
    pa=r+R*j.parent_point_body; pb=r+a+AR*j.child_point_body; mid=(pa+pb)/2
    wa,wb=R*w,AR*aw
    va=v+cross(wa,mid-r); vb=av+cross(wb,mid-r-a)
    slip=vb-va
    damping_power=-dot(slip,j.c_translation_n_s_m*slip)-dot(wb-wa,j.c_rotation_n_m_s_rad*(wb-wa))
    theta=contact_angle(contact_quaternion_product(q,j.rest_child_parent_quat),aq)
    kinetic_energy=sum((b.mass_kg*dot(s.velocity_m_s,s.velocity_m_s)+
        dot(s.angular_velocity_body_rad_s,b.inertia_body_kg_m2*s.angular_velocity_body_rad_s))/2 for (b,s) in zip(model.bodies,states))
    joint_potential=dot(pb-pa,j.k_translation_n_m*(pb-pa))/2+j.k_rotation_n_m_rad[1,1]*theta^2/2
    energy=kinetic_energy+joint_potential
    P=sum(b.mass_kg*s.velocity_m_s for (b,s) in zip(model.bodies,states))
    H=sum(cross(s.position_m,b.mass_kg*s.velocity_m_s)+
        contact_rotation(s.quaternion)*(b.inertia_body_kg_m2*s.angular_velocity_body_rad_s) for (b,s) in zip(model.bodies,states))
    local_center=R'*a; delta=abs.(local_center)-rt.config.bus_half_size
    gap=norm(max.(delta,0.))+min(maximum(delta),0.)-rt.config.bumper_radius
    offset=local_center-clamp.(local_center,-rt.config.bus_half_size,rt.config.bus_half_size)
    normal_local=offset/norm(offset)
    gap_rate=dot(normal_local,R'*SVector{3,Float64}(sv.att_v[:,1])-cross(w,local_center))
    fcon=copy(B.efc_force(rt.data)); vcon=copy(B.efc_vel(rt.data))
    normal=isempty(fcon) ? 0. : fcon[1]
    ft=length(fcon)==3 ? norm(fcon[2:3]) : 0.
    slip_speed=length(vcon)==3 ? norm(vcon[2:3]) : 0.
    cone_ratio=normal>1e-9 && rt.config.friction>0 ? ft/(rt.config.friction*normal) : 0.
    friction_power=length(fcon)==3 ? dot(fcon[2:3],vcon[2:3]) : 0.
    power_projection_error=abs(contact_power-dot(fcon,vcon))
    pair_force=norm(loads[1].force_n+loads[2].force_n)
    pair_moment=norm(loads[1].torque_nm+loads[2].torque_nm+cross(a,loads[2].force_n))
    return (;time=t,gap,gap_rate,normal,cone_ratio,slip_speed,friction_power,contact_power,damping_power,
        energy,kinetic_energy,joint_potential,P,H,pair_force,pair_moment,power_projection_error,loads,states,relative_angle=theta)
end

function contact_trajectory_ledger(args,sol,contact; sample_step=2.0^-11)
    C=SpaceAGORA.ContactLoadServices
    model=contact_analysis_model(args)
    bodies=Tuple(C.ContactBody(b.mass_kg,b.inertia_body_kg_m2) for b in model.bodies)
    rt=C.contact_prepare(contact,bodies)
    origin=SVector{3,Float64}(first(sol.u).sc[1].pos)
    # A constant center-of-mass velocity keeps boosted kinetic-energy ledgers
    # away from cancellation. This is a Galilean transformation, not a moving bus frame.
    frame_velocity=SVector{3,Float64}(first(sol.u).sc[1].vel)+SVector{3,Float64}(first(sol.u).sc[1].att_v[:,1])/21
    sample(t)=contact_observation(model,sol(t).sc[1],t,origin,frame_velocity,rt)
    try
        times=collect(range(first(sol.t),last(sol.t);length=ceil(Int,(last(sol.t)-first(sol.t))/sample_step)+1))
        rows=sample.(times)
        nodes=(-.8611363115940526,-.3399810435848563,.3399810435848563,.8611363115940526)
        weights=(.3478548451374538,.6521451548625461,.6521451548625461,.3478548451374538)
        events=NamedTuple[]
        function quad(a,b)
            m=(a+b)/2; h=(b-a)/2
            sum(map(nodes,weights) do n,w
                s=sample(m+h*n)
                h*w*SVector(s.contact_power,s.damping_power,s.normal,s.friction_power,s.loads[2].force_n...,s.loads[2].torque_nm...)
            end)
        end
        function piece(a,b,ga,gb)
            if ga*gb<0
                lo,hi=a,b
                for _ in 1:40
                    mid=(lo+hi)/2
                    if (sample(mid).gap>0)==(ga>0)
                        lo=mid
                    else
                        hi=mid
                    end
                end
                root=(lo+hi)/2
                push!(events,(time=root,entering=ga>0,gap_rate=sample(root).gap_rate))
                return quad(a,root)+quad(root,b)
            end
            return quad(a,b)
        end
        cumulative=[zero(SVector{10,Float64})]
        for i in 1:length(times)-1
            a,b=times[i],times[i+1]; m=(a+b)/2; gm=sample(m).gap
            integral=piece(a,m,rows[i].gap,gm)+piece(m,b,gm,rows[i+1].gap)
            push!(cumulative,last(cumulative)+integral)
        end
        energy_residual=maximum(abs(row.energy-rows[1].energy-work[1]-work[2]) for (row,work) in zip(rows,cumulative))
        active=filter(s->s.normal>1e-5,rows)
        slide=filter(s->s.normal>1e-5&&s.cone_ratio>.99&&s.slip_speed>.01,rows)
        slow=filter(s->s.normal>1e-5&&s.cone_ratio<.95&&s.slip_speed<.001,rows)
        metrics=Dict{String,Any}(
            "maximum_penetration_m"=>max(0.,-minimum(s.gap for s in rows)),
            "peak_normal_force_N"=>maximum(s.normal for s in rows),
            "normal_impulse_N_s"=>last(cumulative)[3],
            "bumper_contact_impulse_N_s"=>collect(last(cumulative)[5:7]),
            "bumper_contact_angular_impulse_N_m_s"=>collect(last(cumulative)[8:10]),
            "energy_work_residual_J"=>energy_residual,
            "contact_work_J"=>last(cumulative)[1],"damping_work_J"=>last(cumulative)[2],
            "friction_work_J"=>last(cumulative)[4],"normal_contact_work_J"=>last(cumulative)[1]-last(cumulative)[4],
            "actuation_work_J"=>0.,"external_work_J"=>0.,
            "power_projection_residual_W"=>maximum(s.power_projection_error for s in rows),
            "mechanical_energy_change_J"=>last(rows).energy-first(rows).energy,
            "momentum_residual_kg_m_s"=>maximum(norm(s.P-rows[1].P) for s in rows),
            "angular_momentum_residual_kg_m2_s"=>maximum(norm(s.H-rows[1].H) for s in rows),
            "force_balance_residual_N"=>maximum(s.pair_force for s in rows),
            "moment_balance_residual_Nm"=>maximum(s.pair_moment for s in rows),
            "maximum_cone_ratio"=>isempty(active) ? 0. : maximum(s.cone_ratio for s in active),
            "sliding_samples"=>length(slide),"low_slip_interior_samples"=>length(slow),
            "maximum_sliding_friction_power_W"=>isempty(slide) ? 0. : maximum(s.friction_power for s in slide),
            "maximum_damping_power_W"=>maximum(s.damping_power for s in rows),
            "rejected_steps"=>sol.stats.nreject,"sample_step_s"=>times[2]-times[1],
            "maximum_relative_joint_angle_rad"=>maximum(s.relative_angle for s in rows),
            "minimum_normal_force_N"=>minimum(s.normal for s in rows),
            "duration_s"=>last(sol.t),
            "sliding_power_qualified"=>!isempty(slide) && maximum(s.friction_power for s in slide)<=1e-10)
        enters=filter(e->e.entering,events); exits=filter(e->!e.entering,events)
        if !isempty(enters) && !isempty(exits)
            incoming=first(enters); outgoing=first(exits)
            metrics["first_touch_time_s"]=incoming.time
            metrics["first_separation_time_s"]=outgoing.time
            metrics["incoming_normal_speed_m_s"]=-incoming.gap_rate
            metrics["outgoing_normal_speed_m_s"]=outgoing.gap_rate
            metrics["effective_normal_restitution"]=outgoing.gap_rate/(-incoming.gap_rate)
        end
        inside=first(rows).gap<0; start=first(times); duration=0.
        for e in events
            if e.entering
                start=e.time; inside=true
            else
                inside && (duration+=e.time-start)
                inside=false
            end
        end
        inside && (duration+=last(times)-start)
        metrics["total_contact_duration_s"]=duration
        return (;metrics,rows,cumulative,events)
    finally
        C.contact_close!(rt)
    end
end

# These are deliberately different physical cases, not alternate parameters
# selected to make the qualification failures disappear.
contact_validation_cases() = (
    (name="head_on",offset=0.,tangent=0.,friction=0.,damping=0.),
    (name="sliding",offset=.2,tangent=.4,friction=.1,damping=0.),
    (name="damped",offset=.2,tangent=.15,friction=.4,damping=.2),
    (name="low_slip",offset=.2,tangent=.01,friction=.8,damping=0.),
    (name="grazing",offset=.42,tangent=.05,friction=.4,damping=0.))

function write_contact_trajectory(path,ledger)
    labels=["time_s","gap_m","gap_rate_m_s","normal_force_N","cone_ratio","slip_speed_m_s",
        "friction_power_W","contact_power_W","joint_damping_power_W","kinetic_energy_J",
        "joint_potential_J","mechanical_energy_J","gravity_potential_J","external_work_J","actuation_work_J",
        "contact_work_J","joint_damping_work_J","normal_impulse_N_s","friction_work_J"]
    append!(labels,["$(prefix)_$(axis)" for prefix in ("P_kg_m_s","H_kg_m2_s","bumper_impulse_N_s","bumper_angular_impulse_N_m_s") for axis in ("x","y","z")])
    for body in ("bus","bumper")
        for (field,axes) in (("r_m",("x","y","z")),("v_m_s",("x","y","z")),
                ("q",("x","y","z","w")),("omega_body_rad_s",("x","y","z")),
                ("smooth_force_N",("x","y","z")),("smooth_torque_N_m",("x","y","z")),
                ("contact_force_N",("x","y","z")),("contact_torque_N_m",("x","y","z")))
            append!(labels,["$(body)_$(field)_$axis" for axis in axes])
        end
    end
    open(path,"w") do io
        println(io,join(labels,','))
        for (s,w) in zip(ledger.rows,ledger.cumulative)
            values=Float64[s.time,s.gap,s.gap_rate,s.normal,s.cone_ratio,s.slip_speed,
                s.friction_power,s.contact_power,s.damping_power,s.kinetic_energy,s.joint_potential,
                s.energy,0.,0.,0.,w[1:4]...,s.P...,s.H...,w[5:10]...]
            for (state,load) in zip(s.states,s.loads)
                append!(values,vcat(state.position_m,state.velocity_m_s,state.quaternion,
                    state.angular_velocity_body_rad_s,state.smooth_force_n,state.smooth_torque_nm,
                    load.force_n,load.torque_nm))
            end
            length(values)==length(labels) || error("Trajectory schema mismatch")
            println(io,join(values,','))
        end
    end
end
