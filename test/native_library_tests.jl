# This file uses only standard libraries: it needs neither an installed environment nor the native library,
# and gives the same result on every platform. NativeLibrary.toml (where each platform's MuJoCo comes from)
# and Artifacts.toml (what Pkg selects by os/arch) must describe the same platforms, and the installer's
# native step must follow NativeLibrary.toml.
using Test, TOML
include(joinpath(@__DIR__, "..", "scripts", "setup_env.jl"))
using .CompanionInstallation: native_action, native_platform_key

@testset "native library manifests" begin
    root = dirname(@__DIR__)
    native = TOML.parsefile(joinpath(root, "NativeLibrary.toml"))
    artifacts = TOML.parsefile(joinpath(root, "Artifacts.toml"))
    platforms = native["platforms"]
    @test native["mujoco_version"] == "3.11.0"
    @test Set(keys(platforms)) == Set(["linux_x86_64", "macos_aarch64"])
    # Every table names an artifact entry of the same os/arch, and every artifact entry has a table.
    @test Set(table["artifact"] for table in values(platforms)) == Set(keys(artifacts))
    for (key, table) in platforms
        @testset "$key" begin
            entry = artifacts[table["artifact"]]
            @test key == string(entry["os"], "_", entry["arch"])
            @test occursin(r"^[0-9a-f]{40}$", entry["git-tree-sha1"])
            @test entry["lazy"] === true
            # Pkg reads any other key of an entry as a platform tag, which would break platform matching.
            @test issubset(keys(entry), ["os", "arch", "git-tree-sha1", "lazy", "download"])
            @test table["source"] in ("artifact", "dmg")
            if table["source"] == "artifact"
                @test startswith(only(entry["download"])["url"], "https://github.com/google-deepmind/mujoco/releases/download/3.11.0/")
            else
                @test !haskey(entry, "download")   # Pkg cannot unpack a disk image
                @test startswith(table["url"], "https://github.com/google-deepmind/mujoco/releases/download/3.11.0/")
                @test endswith(table["url"], ".dmg")
                @test occursin(r"^[0-9a-f]{64}$", table["sha256"])
                @test first(split(table["library"], '/')) in table["copy"]
            end
        end
    end
    # The Linux entry is the released one.
    linux = artifacts["mujoco_3_11_0_linux_x86_64"]
    @test linux["git-tree-sha1"] == "6a34a368d528ad0332f9da97de3d2f105977bac8"
    @test only(linux["download"])["sha256"] == "681a9353b296a06682e9e757d6d21ab7a0a5a5bdb218a1c05cb9a38536817c79"
    @test platforms["macos_aarch64"]["sha256"] == "54933aa1a22a212134eaa663e83f4e81d7d2f14d0a778d0ba4fa14f66633b4f1"
end

@testset "installer native step" begin
    @test native_platform_key() in ("linux_x86_64", "macos_aarch64") || native_action() === :unsupported
    @test native_action("linux_x86_64") === :lazy                       # Linux setup downloads nothing by default
    @test native_action("linux_x86_64"; requested=true) === :prefetch
    @test native_action("macos_aarch64") === :prefetch                  # the disk-image build happens at setup
    @test native_action("macos_aarch64"; requested=true) === :prefetch
    for key in ("macos_x86_64", "linux_aarch64", "nt_x86_64")
        @test native_action(key) === :unsupported
        @test native_action(key; requested=true) === :unsupported
    end
    # Arguments are refused before anything is planned or installed.
    for arguments in (String[], ["a", "b"], ["--prefetch-native"], ["a", "--unknown"])
        @test_throws ArgumentError CompanionInstallation.main(arguments)
    end
end
