include("compare_contact_platforms.jl")

function compare_expanded_contact_platforms(left,right)
    a,b=map(dir->TOML.parsefile(joinpath(dir,"summary.toml")),(left,right))
    for key in ("schema_version","core","companion","source_sha256","native_version","contract_version","duration_s","scales","solver")
        a[key]==b[key] || error("Incompatible expanded qualification inputs: $key")
    end
    a["core"]["dirty"]==a["companion"]["dirty"]==false || error("Require clean source revisions")
    keys(a["cases"])==keys(b["cases"]) || error("Different expanded fixture sets")
    result=Dict{String,Any}("schema_version"=>1,"core"=>a["core"],"companion"=>a["companion"],
        "source_sha256"=>a["source_sha256"],"left_platform"=>a["platform"],"right_platform"=>b["platform"],
        "physically_qualified"=>a["qualified"]&&b["qualified"],"cases"=>Dict{String,Any}())
    for name in sort!(collect(keys(a["cases"])))
        ca,cb=a["cases"][name],b["cases"][name]
        ca["trajectory_file"]==cb["trajectory_file"] || error("Different trajectory files")
        fa,fb=last(ca["refinements"]),last(cb["refinements"])
        fa["core_maximum_step_s"]==fb["core_maximum_step_s"]==2.0^-13 || error("Require finest completed refinements")
        paths=map(dir->joinpath(dir,ca["trajectory_file"]),(left,right));x,y=read_contact_csv.(paths)
        keys(x)==keys(y)&&x["time_s"]==y["time_s"] || error("Different trajectory grids or body layouts")
        trajectory=0.
        count=length(filter(k->endswith(k,"_r_x"),collect(keys(x))))
        for k in eachindex(x["time_s"]), body in 1:count
            for (field,scale) in (("r",1.),("v",.25),("w",.25))
                d=[x["body$(body)_$(field)_$axis"][k]-y["body$(body)_$(field)_$axis"][k] for axis in ("x","y","z")]
                trajectory=max(trajectory,norm(d)/scale)
            end
            qa=[x["body$(body)_q_$axis"][k] for axis in ("x","y","z","w")]
            qb=[y["body$(body)_q_$axis"][k] for axis in ("x","y","z","w")]
            dot(qa,qb)<0&&(qb=-qb)
            trajectory=max(trajectory,4atan(norm(qa-qb),norm(qa+qb)))
        end
        impulse_key=startswith(name,"arm_") ? "bus_contact_impulse_N_s" : "bumper_contact_impulse_N_s"
        impulse=norm(fa[impulse_key]-fb[impulse_key])/5.25
        work=abs(fa["contact_work_J"]-fb["contact_work_J"])
        ta,tb=fa["transitions"],fb["transitions"]
        transitions=length(ta)==length(tb)&&all(x["sticking"]==y["sticking"]&&abs(x["time_s"]-y["time_s"])<1e-3 for (x,y) in zip(ta,tb))
        result["cases"][name]=Dict("scaled_trajectory_difference"=>trajectory,"scaled_impulse_difference"=>impulse,
            "contact_work_difference_J"=>work,"transition_agreement"=>transitions,
            "left_trajectory_sha256"=>bytes2hex(sha256(read(paths[1]))),"right_trajectory_sha256"=>bytes2hex(sha256(read(paths[2]))),
            "passed"=>trajectory<1e-3&&impulse<1e-3&&work<1.3125e-6&&transitions)
    end
    result["cross_platform_agreement"]=all(c["passed"] for c in values(result["cases"]))
    result
end
if abspath(PROGRAM_FILE)==@__FILE__
    length(ARGS)==3 || error("Usage: compare_expanded_contact_platforms.jl LEFT RIGHT OUTPUT.toml")
    result=compare_expanded_contact_platforms(ARGS[1],ARGS[2])
    open(io->TOML.print(io,result;sorted=true),ARGS[3],"w")
    println("Expanded platform agreement: ",result["cross_platform_agreement"])
    result["physically_qualified"]&&result["cross_platform_agreement"]||exit(1)
end
