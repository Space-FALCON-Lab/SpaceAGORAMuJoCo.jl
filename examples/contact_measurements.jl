# Independent scalar soft-contact equation for the head-on fixture only.
function scalar_headon_force(x,v)
    x >= .6 && return 0.
    A=1.05; d=.9; tau=.01
    smooth_acceleration=-A*200*(x-.61)
    reference_acceleration=-2/(d*tau)*v-(x-.6)/(d*tau^2)
    return max(0.,(reference_acceleration-smooth_acceleration)/(A/d))
end
function scalar_headon_reference(duration; h=2.0^-16)
    x=SVector(.61,-.25)
    rhs(y)=SVector(y[2],1.05*(-200*(y[1]-.61)+scalar_headon_force(y...)))
    steps=ceil(Int,duration/h); dt=duration/steps
    for _ in 1:steps
        k1=rhs(x); k2=rhs(x+dt/2*k1); k3=rhs(x+dt/2*k2); k4=rhs(x+dt*k3)
        x+=dt/6*(k1+2k2+2k3+k4)
    end
    return x
end

function headon_measurements(sol; sample_step=2.0^-13)
    times=range(first(sol.t),last(sol.t);length=ceil(Int,(last(sol.t)-first(sol.t))/sample_step)+1)
    states=[sol(t).sc[1] for t in times]
    relative=[SVector(s.att_r[1,1],s.att_v[1,1]) for s in states]
    force=[scalar_headon_force(x...) for x in relative]
    power=[f*x[2] for (f,x) in zip(force,relative)]
    energy=[10sum(abs2,s.vel)+.5sum(abs2,s.vel+s.att_v[:,1])+100*(s.att_r[1,1]-.61)^2 for s in states]
    momentum=[21SVector{3,Float64}(s.vel)+SVector{3,Float64}(s.att_v[:,1]) for s in states]
    dt=step(times)
    # Split geometric onset/separation before quadrature: soft contact has a
    # finite damping-force jump at first touch. Uniform trapezoids hide that
    # event error and do not provide monotone work convergence.
    nodes=(-.8611363115940526,-.3399810435848563,.3399810435848563,.8611363115940526)
    weights=(.3478548451374538,.6521451548625461,.6521451548625461,.3478548451374538)
    sample(t)=(s=sol(t).sc[1]; SVector(s.att_r[1,1],s.att_v[1,1]))
    function quadrature(a,b)
        mid=(a+b)/2; radius=(b-a)/2
        values=map(nodes,weights) do n,w
            x=sample(mid+radius*n); f=scalar_headon_force(x...)
            radius*w*SVector(f,f*x[2])
        end
        return sum(values)
    end
    integrals=map(1:length(times)-1) do i
        a,b=times[i],times[i+1]
        if (relative[i][1]-.6)*(relative[i+1][1]-.6)<0
            lo,hi=a,b; sign0=relative[i][1]>.6
            for _ in 1:40
                mid=(lo+hi)/2
                if (sample(mid)[1]>.6)==sign0
                    lo=mid
                else
                    hi=mid
                end
            end
            root=(lo+hi)/2
            quadrature(a,root)+quadrature(root,b)
        else
            quadrature(a,b)
        end
    end
    work=cumsum(vcat(0.,last.(integrals)))
    return Dict("maximum_penetration_m"=>max(0.,.6-minimum(x[1] for x in relative)),
        "peak_contact_force_N"=>maximum(force),"normal_impulse_N_s"=>sum(first,integrals),
        "contact_duration_s"=>dt*count(>(0.),force),
        "momentum_residual_kg_m_s"=>maximum(norm(p-first(momentum)) for p in momentum),
        "energy_work_residual_J"=>maximum(abs.(energy.-first(energy).-work)),
        "net_contact_work_J"=>last(work),"mechanical_energy_change_J"=>last(energy)-first(energy),
        "final_relative_state"=>collect(last(relative)),"sample_step_s"=>dt)
end
