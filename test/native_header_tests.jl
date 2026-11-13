module NativeHeaderTests
using Test
using ..SpaceAGORAMuJoCo: Binding

@testset "native ABI: compiled pinned headers" begin
    compiler=Sys.which("cc")
    if compiler === nothing
        get(ENV,"SPACEAGORA_MUJOCO_REQUIRED","")=="1" &&
            error("mandatory native ABI tests require a C compiler")
        @test_skip false
    else
        libdir=dirname(realpath(Binding.library_path()))
        headers=Sys.isapple() ? joinpath(libdir,"Headers") : joinpath(dirname(libdir),"include","mujoco")
        @test isfile(joinpath(headers,"mujoco.h"))
        mktempdir() do dir
            symlink(headers,joinpath(dir,"mujoco"))
            source=joinpath(dir,"offsets.c"); executable=joinpath(dir,"offsets")
            types=(model="mjModel",data="mjData",opt="mjOption",contact="mjContact")
            expected=Dict{String,Int}()
            statements=String[]
            for (group,fields) in pairs(Binding.OFFSETS), (field,offset) in pairs(fields)
                key="$group.$field"
                expected[key]=offset
                push!(statements,"printf(\"$key=%zu\\n\", offsetof($(getproperty(types,group)), $field));")
            end
            for (name,value) in (("mjModel",Binding.SIZEOF_MJMODEL),
                    ("mjData",Binding.SIZEOF_MJDATA),("mjOption",Binding.SIZEOF_MJOPTION),
                    ("mjContact",Binding.SIZEOF_MJCONTACT))
                expected["sizeof.$name"]=value
                push!(statements,"printf(\"sizeof.$name=%zu\\n\", sizeof($name));")
            end
            write(source,"#include <stdio.h>\n#include <stddef.h>\n#include <mujoco/mujoco.h>\nint main(void) {\n" *
                join(statements,"\n") * "\nreturn 0;\n}\n")
            run(`$compiler -I$dir $source -o $executable`)
            actual=Dict(String(k)=>parse(Int,v) for (k,v) in
                (split(line,'=') for line in split(chomp(read(`$executable`,String)),'\n')))
            for (key,offset) in expected
                @test actual[key]==offset
            end
        end
    end
end
end
