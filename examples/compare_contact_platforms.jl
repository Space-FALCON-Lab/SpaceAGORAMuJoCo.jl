# Compare the saved finest trajectories from two validation artifact folders.
# Julia standard libraries only; this does not rerun or qualify the force model.
using LinearAlgebra, SHA, TOML

function read_contact_csv(path)
    lines=readlines(path)
    length(lines)>2 || error("Empty contact trajectory: $path")
    header=split(first(lines),',')
    rows=[parse.(Float64,split(line,',')) for line in lines[2:end]]
    all(row->length(row)==length(header) && all(isfinite,row),rows) ||
        error("Malformed or nonfinite contact trajectory: $path")
    return Dict(name=>[row[i] for row in rows] for (i,name) in enumerate(header))
end

function compare_contact_platforms(left,right)
    reports=map(dir->TOML.parsefile(joinpath(dir,"summary.toml")),(left,right))
    a,b=reports
    for key in ("schema_version","core","companion","source_sha256","frame",
                "native_version","contact_contract_version","scales","core_solver","native_solver")
        a[key]==b[key] || error("Incompatible validation inputs: $key")
    end
    a["core"]["dirty"]==a["companion"]["dirty"]==false || error("Require clean source revisions")
    keys(a["cases"])==keys(b["cases"]) || error("Different fixture sets")
    scales=a["scales"]
    result=Dict{String,Any}("schema_version"=>1,"core"=>a["core"],"companion"=>a["companion"],
        "source_sha256"=>a["source_sha256"],"scales"=>scales,
        "left"=>Dict("platform"=>a["platform"],"julia_version"=>a["julia_version"],
            "summary_sha256"=>bytes2hex(sha256(read(joinpath(left,"summary.toml")))),
            "qualification_failures"=>a["qualification_failures"]),
        "right"=>Dict("platform"=>b["platform"],"julia_version"=>b["julia_version"],
            "summary_sha256"=>bytes2hex(sha256(read(joinpath(right,"summary.toml")))),
            "qualification_failures"=>b["qualification_failures"]),
        "trajectory_budget"=>1e-3,"impulse_budget"=>1e-3,"cases"=>Dict{String,Any}())
    vector(row,prefix,i)=Float64[row[prefix*axis][i] for axis in ("x","y","z")]
    quaternion(row,prefix,i)=normalize(Float64[row[prefix*axis][i] for axis in ("x","y","z","w")])
    for name in sort!(collect(keys(a["cases"])))
        ca,cb=a["cases"][name],b["cases"][name]
        for key in ("mount_offset_m","initial_tangential_velocity_m_s","friction_coefficient",
                    "joint_translation_damping_N_s_m","trajectory_file")
            ca[key]==cb[key] || error("Different $name fixture: $key")
        end
        fine_a=last(filter(r->r["status"]=="completed",ca["refinements"]))
        fine_b=last(filter(r->r["status"]=="completed",cb["refinements"]))
        fine_a["core_maximum_step_s"]==fine_b["core_maximum_step_s"]==2.0^-13 ||
            error("Require completed finest-step trajectories")
        paths=map(dir->joinpath(dir,ca["trajectory_file"]),(left,right))
        x,y=read_contact_csv.(paths)
        x["time_s"]==y["time_s"] || error("Different sample grids for $name")
        trajectory=0.; impulse=0.; angular_impulse=0.
        for i in eachindex(x["time_s"])
            for body in ("bus","bumper")
                for (suffix,scale) in (("_r_m_",scales["length_m"]),
                    ("_v_m_s_",scales["velocity_m_s"]),
                    ("_omega_body_rad_s_",scales["velocity_m_s"]/scales["length_m"]))
                    trajectory=max(trajectory,norm(vector(x,body*suffix,i)-vector(y,body*suffix,i))/scale)
                end
                qa,qb=quaternion(x,body*"_q_",i),quaternion(y,body*"_q_",i)
                dot(qa,qb)<0 && (qb=-qb)
                trajectory=max(trajectory,4atan(norm(qa-qb),norm(qa+qb)))
            end
            impulse=max(impulse,norm(vector(x,"bumper_impulse_N_s_",i)-
                vector(y,"bumper_impulse_N_s_",i))/scales["momentum_kg_m_s"])
            angular_impulse=max(angular_impulse,norm(vector(x,"bumper_angular_impulse_N_m_s_",i)-
                vector(y,"bumper_angular_impulse_N_m_s_",i))/scales["angular_momentum_kg_m2_s"])
        end
        result["cases"][name]=Dict("scaled_trajectory_difference"=>trajectory,
            "scaled_impulse_difference"=>impulse,"scaled_angular_impulse_difference"=>angular_impulse,
            "contact_work_difference_J"=>maximum(abs.(x["contact_work_J"]-y["contact_work_J"])),
            "samples"=>length(x["time_s"]),"duration_s"=>last(x["time_s"]),
            "left_trajectory_sha256"=>bytes2hex(sha256(read(paths[1]))),
            "right_trajectory_sha256"=>bytes2hex(sha256(read(paths[2]))),
            "passed"=>trajectory<1e-3 && impulse<1e-3 && angular_impulse<1e-3)
    end
    result["cross_platform_agreement"]=all(c["passed"] for c in values(result["cases"]))
    result["physically_qualified"]=a["qualified"] && b["qualified"]
    return result
end

if abspath(PROGRAM_FILE)==@__FILE__
    length(ARGS)==3 || error("Usage: julia compare_contact_platforms.jl LEFT_DIR RIGHT_DIR OUTPUT.toml")
    result=compare_contact_platforms(ARGS[1],ARGS[2])
    open(io->TOML.print(io,result;sorted=true),ARGS[3],"w")
    println("Cross-platform numerical agreement: ",result["cross_platform_agreement"])
    println("Physical qualification: ",result["physically_qualified"])
    result["cross_platform_agreement"] || exit(1)
end
