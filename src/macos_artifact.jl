# Included by binding.jl only. macOS arm64: the official MuJoCo release is a disk image, which Pkg cannot
# unpack, so the artifact named in Artifacts.toml has a pinned tree hash and no download. It is built here
# from the official dmg, whose URL and sha256 are in NativeLibrary.toml. Linux never runs this file's code.

const NATIVE_LIBRARY_TOML = joinpath(@__DIR__, "..", "NativeLibrary.toml")
const ARTIFACTS_TOML = joinpath(@__DIR__, "..", "Artifacts.toml")

"""Name of this host's table in `NativeLibrary.toml`, such as `linux_x86_64` or `macos_aarch64`."""
platform_key() = string(Sys.isapple() ? "macos" : Sys.islinux() ? "linux" : lowercase(string(Sys.KERNEL)), "_", Sys.ARCH)

@noinline _unsupported_platform() = throw(ErrorException(
    "SpaceAGORAMuJoCo: unsupported platform $(Sys.KERNEL)/$(Sys.ARCH). " *
    "MuJoCo 3.11.0 is pinned for Linux x86_64 and macOS arm64 only."))

"""This host's table in `NativeLibrary.toml`; throws the unsupported-platform error when there is none."""
function native_platform()
    platforms = get(TOML.parsefile(NATIVE_LIBRARY_TOML), "platforms", Dict{String, Any}())
    haskey(platforms, platform_key()) || _unsupported_platform()
    return platforms[platform_key()]
end

"""
Directory of the macOS artifact, built from the official disk image the first time. An artifact directory
that already exists is returned as it is, even if empty: a missing library is then reported, not replaced.
"""
function _macos_artifact_root(spec = native_platform())
    hash = artifact_hash(spec["artifact"], ARTIFACTS_TOML)
    hash === nothing && error("Artifacts.toml has no $(spec["artifact"]) entry for $(platform_key())")
    artifact_exists(hash) && return artifact_path(hash)
    return _build_macos_artifact(spec, hash)
end

function _build_macos_artifact(spec, expected)
    work = mktempdir()
    try
        dmg = joinpath(work, basename(spec["url"]))
        @info "SpaceAGORAMuJoCo: downloading the official MuJoCo disk image (one time)" url = spec["url"]
        Downloads.download(spec["url"], dmg)
        actual = bytes2hex(open(SHA.sha256, dmg))
        if actual != spec["sha256"]
            rm(dmg; force = true)
            error("MuJoCo disk image checksum mismatch: NativeLibrary.toml pins sha256 $(spec["sha256"]), " *
                "but $(spec["url"]) gave $actual. Nothing was installed.")
        end
        mountpoint = mkpath(joinpath(work, "mount"))
        run(pipeline(`hdiutil attach -nobrowse -readonly -noautoopen -mountpoint $mountpoint $dmg`; stdout = devnull))
        built = try
            # create_artifact works in a temporary directory and moves it into place atomically, so a
            # failure in here leaves no partial artifact. `cp -R` keeps the framework's symbolic links.
            Pkg.Artifacts.create_artifact() do directory
                for item in spec["copy"]
                    run(`cp -R $(joinpath(mountpoint, item)) $(joinpath(directory, item))`)
                end
            end
        finally
            success(pipeline(`hdiutil detach $mountpoint -quiet`; stderr = devnull)) ||
                run(`hdiutil detach $mountpoint -force -quiet`)
        end
        if built != expected
            Pkg.Artifacts.remove_artifact(built)
            error("The MuJoCo artifact built from the verified disk image has git-tree-sha1 " *
                "$(bytes2hex(built.bytes)), but Artifacts.toml pins $(bytes2hex(expected.bytes)). Nothing was installed.")
        end
        return artifact_path(expected)
    finally
        rm(work; recursive = true, force = true)
    end
end
