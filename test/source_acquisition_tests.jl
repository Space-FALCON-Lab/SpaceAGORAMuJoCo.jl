# Tiny local Git repositories only: no remote network, Pkg, native artifact or simulation.
using Test, TOML
include(joinpath(@__DIR__, "..", "scripts", "setup_env.jl"))
const Install = CompanionInstallation
git(args...) = Install.source_git(args...)

function commit_fixture(repo, message)
    run(git("-C", repo, "add", "."))
    run(git("-C", repo, "-c", "user.name=Companion fixture", "-c", "user.email=fixture@example.invalid",
        "commit", "--quiet", "-m", message))
    return readchomp(git("-C", repo, "rev-parse", "HEAD"))
end

@testset "exact managed core acquisition" begin
    mktempdir() do directory
        origin = joinpath(directory, "origin")
        run(git("init", "--quiet", origin))
        write(joinpath(origin, "Project.toml"), "name = \"SpaceAGORA\"\nuuid = \"afbfb69f-5c0b-4832-b760-43725dff8540\"\n")
        first = commit_fixture(origin, "initial core fixture")
        write(joinpath(origin, ".gitmodules"), "[submodule \"native\"]\n\tpath = data/native\n\turl = https://example.invalid/never-fetch.git\n")
        mkpath(joinpath(origin, "data", "native"))
        run(git("-C", origin, "update-index", "--add", "--cacheinfo", "160000,$first,data/native"))
        pointer = "version https://git-lfs.github.com/spec/v1\noid sha256:$(repeat("1", 64))\nsize 123\n"
        write(joinpath(origin, ".gitattributes"), "*.bin filter=lfs diff=lfs merge=lfs -text\n")
        write(joinpath(origin, "native.bin"), pointer)
        pin = commit_fixture(origin, "inert submodule and LFS pointer")
        @test startswith(readchomp(git("-C", origin, "ls-tree", pin, "data/native")), "160000 commit $first")
        env = joinpath(directory, "environment")
        plan = (; destination=env, core=(; path=nothing, url=origin, revision=pin))
        active = Base.active_project()
        acquired = Install.acquire_core(plan)
        @test acquired == joinpath(env, "sources", "SpaceAGORA-" * pin)
        @test readchomp(git("-C", acquired, "rev-parse", "HEAD")) == pin
        @test read(joinpath(acquired, "native.bin"), String) == pointer
        @test !ispath(joinpath(acquired, "data", "native", ".git"))
        @test !ispath(joinpath(acquired, ".git", "modules"))
        @test isempty(readchomp(git("-C", acquired, "status", "--porcelain=v1", "--ignore-submodules=none")))
        @test Base.active_project() == active
        @test !isfile(joinpath(env, "Project.toml"))
        marker = joinpath(acquired, ".git", "spaceagora-companion-source.toml")
        marker_before = read(marker)
        @test Install.acquire_core(plan) == acquired
        @test read(marker) == marker_before

        write(joinpath(acquired, "unexpected.txt"), "keep this data")
        @test_throws ArgumentError Install.acquire_core(plan)
        @test read(joinpath(acquired, "unexpected.txt"), String) == "keep this data"
        rm(joinpath(acquired, "unexpected.txt"))
        write(joinpath(acquired, "Project.toml"), "changed content")
        @test_throws ArgumentError Install.acquire_core(plan)
        run(git("-C", acquired, "checkout", "--", "Project.toml"))
        run(git("-C", acquired, "fetch", "--quiet", "origin", first))
        run(git("-C", acquired, "checkout", "--quiet", "--detach", first))
        @test_throws ArgumentError Install.acquire_core(plan)
        @test readchomp(git("-C", acquired, "rev-parse", "HEAD")) == first
        run(git("-C", acquired, "checkout", "--quiet", "--detach", pin))
        run(git("-C", acquired, "remote", "set-url", "origin", "wrong-source"))
        @test_throws ArgumentError Install.acquire_core(plan)
        run(git("-C", acquired, "remote", "set-url", "origin", origin))
        rm(marker)
        @test_throws ArgumentError Install.acquire_core(plan)
        write(marker, marker_before)

        bad = merge(plan, (; core=(; path=nothing, url=origin, revision=repeat("f", 40))))
        @test_throws ProcessFailedException Install.acquire_core(bad)
        @test !ispath(joinpath(env, "sources", "SpaceAGORA-" * repeat("f", 40)))
        @test sort(readdir(joinpath(env, "sources"))) == ["SpaceAGORA-" * pin]
        @test Install.acquire_core(plan) == acquired

        write(joinpath(origin, "Project.toml"), "name = \"WrongCore\"\nuuid = \"afbfb69f-5c0b-4832-b760-43725dff8540\"\n")
        wrong_package = commit_fixture(origin, "wrong package identity")
        wrong = merge(plan, (; core=(; path=nothing, url=origin, revision=wrong_package)))
        @test_throws ArgumentError Install.acquire_core(wrong)
        @test !ispath(joinpath(env, "sources", "SpaceAGORA-" * wrong_package))
        @test sort(readdir(joinpath(env, "sources"))) == ["SpaceAGORA-" * pin]
        @test !isfile(joinpath(env, "Project.toml"))

        other_env = joinpath(directory, "symlink-environment")
        mkpath(other_env)
        symlink(joinpath(env, "sources"), joinpath(other_env, "sources"))
        @test_throws ArgumentError Install.acquire_core(merge(plan, (; destination=other_env)))
        link_env = joinpath(directory, "symlink-checkout")
        mkpath(joinpath(link_env, "sources"))
        symlink(acquired, joinpath(link_env, "sources", "SpaceAGORA-" * pin))
        @test_throws ArgumentError Install.acquire_core(merge(plan, (; destination=link_env)))

        # A caller-owned checkout override bypasses acquisition and never creates
        # source directories or changes the caller's repository, even when dirty.
        write(joinpath(origin, "caller-owned.txt"), "unchanged")
        override_env = joinpath(directory, "override")
        override = (; destination=override_env, core=(; path=origin, url=nothing, revision=nothing))
        @test Install.acquire_core(override) == origin
        @test !ispath(override_env)
        @test read(joinpath(origin, "caller-owned.txt"), String) == "unchanged"
    end
end
