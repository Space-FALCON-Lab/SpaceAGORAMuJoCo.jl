@testset "free-body agreement with ordinary SpaceAGORA spacecraft: $name" for (name, effectors) in (
        ("point mass", (SM.InverseSquaredGravityModel(),)),
        ("J2", (SM.InverseSquaredJ2GravityModel(),)))
    tend = round(PERIOD / 50) * 50          # one orbit, on a 50 s grid
    every = 50.0
    ref = samples_from_table(engine_reference(effectors, tend, every), every, tend)
    dts = [0.1, 0.05, 0.025, 0.0125]
    diffs = [max_differences(scene_samples(effectors, dt, tend, every), ref) for dt in dts]
    for (dt, d) in zip(dts, diffs)
        @info "$name: scene vs engine over one orbit ($(round(tend)) s), dt = $dt s" max_position_diff_m = d.dr max_velocity_diff_mps = d.dv max_target_position_diff_m = d.dr_target max_relative_position_diff_m = d.drel max_relative_velocity_diff_mps = d.dvrel relative_position = d.dr / 7.0e6
    end
    # Self-convergence by dt halving: the scene's own error (the step holds each wrench for dt) is first
    # order, so successive halvings should roughly halve the difference to the engine.
    ratios = [diffs[i].dr / diffs[i + 1].dr for i in 1:length(dts) - 1]
    @info "$name: position error ratios for dt halving" ratios
    @test all(r -> 1.7 < r < 2.3, ratios)
    @test diffs[end].dr < TOL_POSITION_M
    @test diffs[end].dv < TOL_VELOCITY_MPS
    @test diffs[end].drel < TOL_RELATIVE_POSITION_M
end

# --- runner properties --------------------------------------------------------------------------------

abs_states(sc) = [scene_body_state(sc, i) for i in 1:2]
run_steps!(sc, n) = (for _ in 1:n; scene_step!(sc); end; sc)
same_state(a, b) = all(x.r == y.r && x.v == y.v && x.q == y.q && x.ω == y.ω for (x, y) in zip(abs_states(a), abs_states(b))) &&
    scene_chief(a) == scene_chief(b) && scene_time(a) == scene_time(b)

@testset "deterministic reset" begin
    sc = make_scene((SM.InverseSquaredJ2GravityModel(),); dt=0.05)
    s0 = abs_states(sc)
    run_steps!(sc, 3000)
    first_run = (abs_states(sc), scene_chief(sc))
    @test first_run[1][1].r != s0[1].r
    scene_reset!(sc)
    @test scene_time(sc) == 0.0
    @test all(x.r ≈ y.r && x.v ≈ y.v for (x, y) in zip(abs_states(sc), s0))
    run_steps!(sc, 3000)
    @test (abs_states(sc), scene_chief(sc)) == first_run          # bit-identical
    # a fresh scene reproduces it too, and the initial absolute states are honored
    sc2 = make_scene((SM.InverseSquaredJ2GravityModel(),); dt=0.05)
    run_steps!(sc2, 3000)
    @test same_state(sc, sc2)
    @test maximum(norm(x.r - SVector(R1)) for x in [s0[1]]) < 1e-6
    @test norm(s0[2].r - R2) < 1e-6 && norm(s0[2].v - V2) < 1e-9
end

@testset "chief starts at the mass-weighted COM" begin
    sc = make_scene((SM.InverseSquaredGravityModel(),); dt=0.05)
    R, V = scene_chief(sc)
    @test R ≈ (20 * R1 + 35 * R2) / 55 atol = 1e-6
    @test V ≈ (20 * V1 + 35 * V2) / 55 atol = 1e-9
end

@testset "state get/set round trip" begin
    sc = make_scene((SM.InverseSquaredJ2GravityModel(),); dt=0.05)
    run_steps!(sc, 700)
    st = scene_state(sc)
    @test st.n == 700
    run_steps!(sc, 900)
    ref = (abs_states(sc), scene_chief(sc), scene_time(sc))
    scene_set_state!(sc, st)
    @test scene_time(sc) == 700 * 0.05
    run_steps!(sc, 900)
    @test (abs_states(sc), scene_chief(sc), scene_time(sc)) == ref
    # copy() gives an independent scene (own mjModel and mjData) at the same state
    scene_set_state!(sc, st)
    cp = copy(sc)
    run_steps!(cp, 900)
    @test sc.n == 700 && cp.n == 1600
    @test (abs_states(cp), scene_chief(cp), scene_time(cp)) == ref
    @test_throws DimensionMismatch scene_set_state!(sc, SceneState(0, st.R, st.V, zeros(3)))
end

@testset "configuration is validated" begin
    kw = (; mjcf_xml=TWO_BODIES, planet=EARTH, gravity_effectors=(SM.InverseSquaredGravityModel(),), initial_states=states())
    @test_throws UndefKeywordError ProximityScene(; kw...)                              # dt is required
    @test_throws ArgumentError ProximityScene(; kw..., dt=0.0)
    @test_throws ArgumentError ProximityScene(; kw..., dt=NaN)
    @test_throws ArgumentError ProximityScene(; kw..., dt=0.05, integrator=:midpoint)
    # RK4 would silently run as Euler on the mj_step1/mj_step2 path, so it is refused
    @test_throws ArgumentError ProximityScene(; kw..., dt=0.05, integrator=:rk4)
    @test_throws ArgumentError ProximityScene(; kw..., mjcf_path="x.xml", dt=0.05)    # both sources
    @test_throws ArgumentError ProximityScene(; planet=EARTH, gravity_effectors=kw.gravity_effectors, initial_states=states(), dt=0.05)
    @test_throws ArgumentError ProximityScene(; kw..., dt=0.05, initial_states=states()[1:1])   # missing body state
    @test_throws ArgumentError ProximityScene(; kw..., dt=0.05, initial_states=[states()[1], states()[1]])
    @test_throws ArgumentError ProximityScene(; kw..., dt=0.05, gravity_effectors=(SM.InverseSquaredJ2GravityModel(),))  # no planet_rotation
    @test_throws ArgumentError ProximityScene(; kw..., dt=0.05, gravity_effectors=())
    # the default integrator is implicitfast
    @test make_scene((SM.InverseSquaredGravityModel(),); dt=0.05).integrator === :implicitfast
    sc = make_scene((SM.InverseSquaredGravityModel(),); dt=0.05)
    @test Binding.timestep(sc.model) == 0.05 && Binding.gravity(sc.model) == (0.0, 0.0, 0.0)
    @test Binding.integrator(sc.model) == Int(Binding.INT_IMPLICITFAST)
    # SpaceAGORA initial conditions map to scene states, attitude and rate included (the convention is
    # derived and tested in engine_tests.jl)
    ic_ok = SM.CartesianInitialCondition(R1, V1)
    @test body_state_from_initial_condition("chaser", ic_ok).r == R1
    @test body_state_from_initial_condition("chaser", ic_ok).q == SVector(1.0, 0.0, 0.0, 0.0)
    ic_rot = SM.CartesianInitialCondition(R1, V1; q=SVector(0.0, 0.0, 0.6, 0.8), ang_vel=SVector(0.0, 0.0, 0.1))
    st = body_state_from_initial_condition("chaser", ic_rot)
    @test st.q ≈ SVector(0.8, 0.0, 0.0, 0.6) && st.ω == SVector(0.0, 0.0, 0.1)
end

@testset "feedback acceleration keeps the chief on a thrusted body" begin
    xml = """<mujoco><worldbody><body name="solo"><freejoint/><inertial pos="0 0 0" mass="20" diaginertia="1 1 1"/></body></worldbody></mujoco>"""
    mk() = ProximityScene(; mjcf_xml=xml, dt=0.05, planet=EARTH, gravity_effectors=(SM.InverseSquaredGravityModel(),),
        initial_states=[SceneBodyState("solo", R1, V1)])
    coast, burn = mk(), mk()
    F = reshape([0.5, 0.0, 0.0], 3, 1)
    for _ in 1:2000
        scene_step!(coast); scene_step!(burn; external_forces=F)
    end
    # the single body is the chief: no relative offset develops, thrust acceleration F/m moves both
    @test abs(burn.xipos[1, 2]) + abs(burn.xipos[2, 2]) + abs(burn.xipos[3, 2]) < 1e-9
    dr = scene_body_state(burn, 1).r - scene_body_state(coast, 1).r
    @test dr[1] ≈ 0.5 * (0.5 / 20) * 100.0^2 rtol = 1e-2
    @test scene_chief(burn)[1] ≈ scene_body_state(burn, 1).r
end

# Memory malloc still has allocated: glibc mallinfo2 `uordblks` (all arenas) plus `hblkhd` (mmapped chunks).
# mallinfo2 needs glibc 2.33 or later; with an older glibc the memory loops below still run but their bounds
# are skipped (mallinfo's int fields wrap at 4 GiB).
const MALLINFO2 = try cglobal(:mallinfo2) catch; C_NULL end
allocated_mb() = MALLINFO2 == C_NULL ? NaN : (mi = ccall(MALLINFO2, NTuple{10, Csize_t}, ()); Int(mi[8] + mi[5]) / 2^20)

@testset "native memory: repeated make/free neither leaks nor crashes" begin
    # The leak signal is the memory malloc still has allocated (`allocated_mb`). Process RSS is not one.
    # Every mjData reserves a large arena that Julia's GC cannot see, so a cycle holds hundreds of them
    # until a collection runs the finalizers; once they are freed glibc keeps the heap pages they touched,
    # and how many depends on where the chunks landed. RSS follows that high-water mark and varies widely
    # between runs with nothing leaked, so it is reported only. Dropping any one of the binding's frees
    # (data, model, spec) leaves at least a model's worth of memory per scene allocated, several times
    # this bound.
    rss_mb() = parse(Int, split(read("/proc/self/statm", String))[2]) * 4096 / 2^20
    cycle() = for _ in 1:150
        sc = make_scene((SM.InverseSquaredGravityModel(),); dt=0.05)
        run_steps!(sc, 3)
        cp = copy(sc)
        run_steps!(cp, 3)
    end
    cycle(); GC.gc(true); GC.gc(true)
    before = (allocated = allocated_mb(), rss = rss_mb())
    for _ in 1:4
        cycle(); GC.gc(true); GC.gc(true)
    end
    growth = allocated_mb() - before.allocated
    @info "Growth over 1200 scene make/free cycles (MB)" allocated = growth rss = rss_mb() - before.rss
    if MALLINFO2 == C_NULL
        @warn "glibc $(unsafe_string(ccall(:gnu_get_libc_version, Cstring, ()))) has no mallinfo2 (2.33 or later): allocated-memory bound not checked"
        @test_skip growth < 4
    else
        @test growth < 4
    end
end

@testset "native memory: close releases each scene at once" begin
    # Without close these scenes stay allocated until a collection runs, and a loop like this one may run
    # none: the peak would grow by a whole mjData per scene. With close it stays flat. The first cycle
    # runs before the baseline so that compiling these calls does not count.
    cycle!(n) = for _ in 1:n
        sc = make_scene((SM.InverseSquaredGravityModel(),); dt=0.05)
        run_steps!(sc, 3)
        cp = copy(sc)
        run_steps!(cp, 3)
        close(cp); close(sc)
    end
    cycle!(1); GC.gc(true); GC.gc(true)
    before = allocated_mb()
    peak = 0.0
    for _ in 1:150
        cycle!(1)
        peak = max(peak, allocated_mb() - before)
    end
    @info "Peak allocated growth over 150 scene make/copy/close cycles without a collection (MB)" peak
    if MALLINFO2 == C_NULL
        @test_skip peak < 4                  # no mallinfo2: warned by the testset above
    else
        @test peak < 4
    end
end

@testset "close: idempotent, and a closed scene refuses every native call" begin
    sc = make_scene((SM.InverseSquaredGravityModel(),); dt=0.05)
    run_steps!(sc, 3)
    cp = copy(sc)
    snapshot = scene_state(cp)
    t, chief, names = scene_time(sc), scene_chief(sc), scene_body_names(sc)
    @test isopen(sc)
    close(sc); close(sc)
    @test !isopen(sc) && !isopen(sc.model) && !isopen(sc.data)
    @test_throws InvalidStateException scene_step!(sc)
    @test_throws InvalidStateException scene_step!(sc; external_forces=zeros(3, 2))
    @test_throws InvalidStateException scene_reset!(sc)
    @test_throws InvalidStateException scene_reset!(sc, states())
    @test_throws InvalidStateException scene_state(sc)
    @test_throws InvalidStateException scene_set_state!(sc, snapshot)
    @test_throws InvalidStateException scene_body_state(sc, 1)
    @test_throws InvalidStateException scene_body_state(sc, "chaser")
    @test_throws InvalidStateException scene_ctrl(sc)
    @test_throws InvalidStateException scene_arena_use(sc)
    @test_throws InvalidStateException copy(sc)
    @test_throws InvalidStateException deepcopy(sc)
    # the binding refuses the closed handles too, so internal paths cannot reach freed memory either
    @test_throws InvalidStateException Binding.forward!(sc.model, sc.data)
    @test_throws InvalidStateException Binding.copy_model(sc.model)
    # no field of the scene points into the freed memory any more
    @test isempty(sc.qpos) && isempty(sc.qvel) && isempty(sc.ctrl) && isempty(sc.mass)
    @test size(sc.xfrc) == (6, 0) && size(sc.xipos) == (3, 0) && size(sc.xquat) == (4, 0)
    # values held on the Julia side stay readable
    @test (scene_time(sc), scene_chief(sc), scene_body_names(sc)) == (t, chief, names)
    # a copy owns its native memory: closing the source does not affect it, nor the reverse
    run_steps!(cp, 3)
    @test cp.n == 6 && isopen(cp)
    branch = copy(cp)
    close(cp)
    run_steps!(branch, 1)
    @test branch.n == 7
    close(branch)
    GC.gc(true); GC.gc(true)                 # the finalizers of closed scenes must not free again
    @test !isopen(branch)
end

@testset "arena use: MJCF size and the peak since the last reset" begin
    small = replace(TWO_BODIES, "<worldbody>" => "<size memory=\"64K\"/>\n  <worldbody>")
    sc = ProximityScene(; mjcf_xml=small, dt=0.05, planet=EARTH, gravity_effectors=(SM.InverseSquaredGravityModel(),),
        initial_states=states())
    use = scene_arena_use(sc)
    @test use.size == 65536
    @test 0 < use.peak <= use.size
    run_steps!(sc, 10)
    # the scene itself behaves the same with the small arena
    ref = run_steps!(make_scene((SM.InverseSquaredGravityModel(),); dt=0.05), 10)
    @test same_state(sc, ref)
    @test scene_arena_use(copy(sc)).size == 65536          # copies keep the arena size
    @test scene_arena_use(ref).size > 65536                 # MuJoCo's default is larger
    # bodies in contact need more arena than the same bodies apart
    touching = replace(TWO_BODIES, "contype=\"0\" conaffinity=\"0\" " => "")
    apart = [SceneBodyState("chaser", R1, V1), SceneBodyState("target", R1 + SVector(5.0, 0.0, 0.0), V1)]
    overlap = [SceneBodyState("chaser", R1, V1), SceneBodyState("target", R1 + SVector(1.08, 0.0, 0.0), V1)]   # 2 cm deep
    mk(st) = ProximityScene(; mjcf_xml=touching, dt=0.01, planet=EARTH, gravity_effectors=(SM.InverseSquaredGravityModel(),),
        initial_states=st)
    a = mk(apart); b = mk(overlap)
    run_steps!(a, 5); run_steps!(b, 5)
    @test scene_arena_use(b).peak > scene_arena_use(a).peak
    scene_reset!(b, apart)                              # mj_resetData clears the peak
    fresh = mk(apart)
    @test scene_arena_use(b).peak == scene_arena_use(fresh).peak
    foreach(close, (sc, ref, a, b, fresh))
end

@testset "MuJoCo instability is an error, not a silent reset" begin
    sc = make_scene((SM.InverseSquaredJ2GravityModel(),); dt=0.05)
    run_steps!(sc, 3)
    forces = zeros(3, 2); forces[1, 1] = Inf       # drives qacc to Inf; MuJoCo would reset the data and go on
    err = try scene_step!(sc; external_forces=forces); nothing catch e; e end
    @test err isa ErrorException
    @test occursin("unstable", err.msg) && occursin("mjWARN_BADQ", err.msg) && occursin("step 4", err.msg)
    # a huge but finite force is caught the same way
    sc = make_scene((SM.InverseSquaredJ2GravityModel(),); dt=0.05)
    forces = fill(1e300, 3, 2)
    @test_throws ErrorException scene_step!(sc; external_forces=forces)
end
