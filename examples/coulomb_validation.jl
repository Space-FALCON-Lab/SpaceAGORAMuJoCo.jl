# Include after bus_bumper_fixture.jl and contact_validation.jl. This ledger
# replays immutable accepted-mode history; it never infers a mode from the last
# native query or silently changes modes while integrating work.
function coulomb_trajectory_ledger(args,sol,contact;sample_step=2.0^-11)
    C=SpaceAGORA.ContactLoadServices
    model=contact_analysis_model(args)
    bodies=Tuple(C.ContactBody(b.mass_kg,b.inertia_body_kg_m2) for b in model.bodies)
    rt=C.contact_prepare(contact,bodies)
    run=sol.prob.p.shared_buffers.contact_runtime[]
    transitions=copy(run.runtime.transitions)
    origin=SVector{3,Float64}(first(sol.u).sc[1].pos)
    velocity=SVector{3,Float64}(first(sol.u).sc[1].vel)+SVector{3,Float64}(first(sol.u).sc[1].att_v[:,1])/21
    function sample(t)
        index=findlast(x->x[1]<=t,transitions)
        rt.sticking=index!==nothing && transitions[index][2]
        row=contact_observation(model,sol(t;continuity=:right).sc[1],t,origin,velocity,rt)
        (;row...,sticking=rt.sticking)
    end
    try
        times=sort!(unique!(vcat(collect(range(first(sol.t),last(sol.t);length=ceil(Int,last(sol.t)/sample_step)+1)),first.(transitions))))
        rows=sample.(times)
        nodes=(-.8611363115940526,-.3399810435848563,.3399810435848563,.8611363115940526)
        weights=(.3478548451374538,.6521451548625461,.6521451548625461,.3478548451374538)
        function rawquad(a,b)
            m=(a+b)/2;h=(b-a)/2
            sum(map(nodes,weights) do x,w
                r=sample(m+h*x)
                h*w*SVector(r.contact_power,r.damping_power,r.friction_power,r.sticking ? r.slip_speed : 0.,r.loads[2].force_n...)
            end)
        end
        scales=SVector(1.3125,1.3125,1.3125,1.,5.25,5.25,5.25)
        function quad(a,b,depth=0)
            m=(a+b)/2;coarse=rawquad(a,b);fine=rawquad(a,m)+rawquad(m,b)
            maximum(abs.((fine-coarse)./scales))<=max(1e-12,1e-9*(b-a)/last(times)) && return fine
            # A supercritical load can reverse slip without an intervening
            # sticking interval; its force jump also needs localization.
            depth<30 || error("Coulomb work quadrature did not converge on [$a,$b]: $(fine-coarse).")
            quad(a,m,depth+1)+quad(m,b,depth+1)
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
                m=(lo+hi)/2;work=quad(a,m)+quad(m,b)
            else
                work=quad(a,b)
            end
            push!(cumulative,last(cumulative)+work)
        end
        accepted=sample.(unique(sol.t))
        stuck=filter(r->r.sticking&&r.normal>1e-5,accepted)
        projection(t)=hasproperty(run,:projections) ? sum((x.work_J for x in run.projections if x.time_s<=t);init=0.) : (t==last(times) ? run.projection_work_J[] : 0.)
        metrics=Dict{String,Any}(
            "duration_s"=>last(sol.t),"transitions"=>[Dict("time_s"=>t,"sticking"=>s) for (t,s) in transitions],
            "capture_speed_m_s"=>contact.capture_speed,"maximum_sticking_speed_m_s"=>maximum((r.slip_speed for r in stuck);init=0.),
            "sticking_slip_distance_m"=>last(cumulative)[4],"sticking_samples"=>length(stuck),
            "maximum_cone_ratio"=>maximum(r.cone_ratio for r in accepted),
            "maximum_friction_power_W"=>maximum(r.friction_power for r in accepted),
            "energy_work_residual_J"=>maximum(abs(r.energy-first(rows).energy-w[1]-w[2]-projection(r.time)) for (r,w) in zip(rows,cumulative)),
            "momentum_residual_kg_m_s"=>maximum(norm(r.P-first(rows).P) for r in rows),
            "angular_momentum_residual_kg_m2_s"=>maximum(norm(r.H-first(rows).H) for r in rows),
            "contact_work_J"=>last(cumulative)[1],"joint_damping_work_J"=>last(cumulative)[2],"friction_work_J"=>last(cumulative)[3],
            "projection_work_J"=>run.projection_work_J[],"projection_count"=>run.projection_count[],
            "bumper_contact_impulse_N_s"=>collect(last(cumulative)[5:7]),
            "force_balance_residual_N"=>maximum(r.pair_force for r in accepted),
            "moment_balance_residual_Nm"=>maximum(r.pair_moment for r in accepted),
            "power_projection_residual_W"=>maximum(r.power_projection_error for r in accepted))
        (;rows,cumulative,accepted,metrics)
    finally
        C.contact_close!(rt)
    end
end
