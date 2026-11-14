# Reduced contact-coordinate benchmark: unit tangential effective mass, fixed
# 10 N normal preload, mu=.4, and a prescribed tangential load. This is a
# constitutive reference problem, not a second integrator in the core adapter.
using LinearAlgebra, StaticArrays, TOML

function friction_creep_case(alpha,vs;duration=2.,step_fraction=.05)
    normal=10.; mu=.4; limit=mu*normal; load=alpha*limit
    W=Matrix{Float64}(I,3,3); Rn=.1; bn=-normal*(W[1,1]+Rn)
    tau=vs/limit
    n=ceil(Int,duration/(step_fraction*tau)); h=duration/n
    function rhs(y)
        f=SpaceAGORAMuJoCo._regularized_contact_force(W,Rn,bn,[y[1],0.],mu,vs)
        SVector(load+f[2],y[1],f[2]*y[1])
    end
    y=SVector(0.,0.,0.); maximum_speed=0.; maximum_power=0.
    for _ in 1:n
        k1=rhs(y); k2=rhs(y+h/2*k1); k3=rhs(y+h/2*k2); k4=rhs(y+h*k3)
        y+=h/6*(k1+2k2+2k3+k4)
        maximum_speed=max(maximum_speed,abs(y[1]))
        maximum_power=max(maximum_power,rhs(y)[3])
    end
    # Independent analytic solution: exponential creep, followed by constant
    # acceleration after reaching the Coulomb limit for a supercritical load.
    a=abs(alpha); t=min(duration,a>1 ? -tau*log1p(-1/a) : duration)
    vr=a*vs*(-expm1(-t/tau)); xr=a*vs*(t+tau*expm1(-t/tau))
    if t<duration
        dt=duration-t; acceleration=(a-1)*limit
        xr+=vr*dt+acceleration*dt^2/2
        vr+=acceleration*dt
    end
    vr*=sign(alpha); xr*=sign(alpha)
    return Dict("load_fraction"=>alpha,"slip_speed_m_s"=>vs,"duration_s"=>duration,
        "maximum_step_s"=>h,"maximum_speed_m_s"=>maximum_speed,
        "displacement_m"=>y[2],"reference_displacement_m"=>xr,
        "final_velocity_m_s"=>y[1],"reference_velocity_m_s"=>vr,
        "velocity_error_m_s"=>abs(y[1]-vr),"displacement_error_m"=>abs(y[2]-xr),
        "maximum_friction_power_W"=>maximum_power,"friction_work_J"=>y[3],
        "energy_work_residual_J"=>abs(y[1]^2/2-load*y[2]-y[3]),
        "subcritical"=>a<1,"creep_speed_budget_m_s"=>vs,
        "creep_displacement_budget_m"=>vs*duration)
end

function friction_creep_report(;duration=2.)
    records=[friction_creep_case(alpha,vs;duration) for vs in (.0005,.001,.002)
        for alpha in (-1.25,-.75,-.25,.25,.75,1.25)]
    passed=all(r["velocity_error_m_s"]<1e-7 && r["displacement_error_m"]<1e-7 &&
        r["energy_work_residual_J"]<1e-8 && r["maximum_friction_power_W"]<=1e-12 &&
        (!r["subcritical"] || (r["maximum_speed_m_s"]<=r["creep_speed_budget_m_s"] &&
            abs(r["displacement_m"])<=r["creep_displacement_budget_m"])) for r in records)
    return Dict("model"=>"passive_regularized_coulomb_v1","qualified"=>passed,
        "normal_preload_N"=>10.,"tangential_effective_mass_kg"=>1.,"friction_coefficient"=>.4,
        "scope"=>"constant-load reduced contact-coordinate reference; approximate sticking with finite creep",
        "cases"=>records)
end
