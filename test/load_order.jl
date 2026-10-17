# Independent fresh-process probes; neither order loads the native artifact.
using Test, Libdl
length(ARGS) == 1 || error("Pass core-first or companion-first")
if only(ARGS) == "core-first"
    using SpaceAGORA
    @test !any(pkg -> pkg.name == "SpaceAGORAMuJoCo", keys(Base.loaded_modules))
    using SpaceAGORAMuJoCo
elseif only(ARGS) == "companion-first"
    using SpaceAGORAMuJoCo
    using SpaceAGORA
else
    error("Pass core-first or companion-first")
end

@testset "versioned external propagation adapter contract" begin
    @test !any(path -> occursin(r"(?i)^(lib)?mujoco([.-]|$)", basename(path)), Libdl.dllist())
    services = SpaceAGORA.ExternalPropagationServices
    @test SpaceAGORAMuJoCo.EP === services
    @test services.CONTRACT_VERSION == SpaceAGORAMuJoCo.CORE_CONTRACT_VERSION == v"2.0.0"
    @test services.require_version(v"2.0.0") === nothing
    for incompatible in (v"0.0.0", v"1.0.0")
        @test_throws services.CompatibilityError services.require_version(incompatible)
    end
    @test SpaceAGORAMuJoCo.ProximitySceneDynamics <: services.AbstractExternalPropagator
    @test services.IMPLEMENTED_FUNCTIONS == (
        :external_spacecraft, :external_step, :external_preflight, :external_prepare,
        :external_sync!, :external_time, :external_state, :external_acceleration)
    @test services.CONSUMED_SERVICES == (
        :AbstractExternalPropagator, :with_external_propagators, :validate_external_propagators,
        :external_runtimes, :external_initial_state, :sample_planet_frame_with_lpi,
        :gravity_backbone_relative_acceleration_ii)
    counts_before = map(name -> length(methods(getproperty(services, name))), services.IMPLEMENTED_FUNCTIONS)
    @test SpaceAGORAMuJoCo.__init__() === nothing
    @test SpaceAGORAMuJoCo.__init__() === nothing
    @test map(name -> length(methods(getproperty(services, name))), services.IMPLEMENTED_FUNCTIONS) == counts_before
    @test !any(path -> occursin(r"(?i)^(lib)?mujoco([.-]|$)", basename(path)), Libdl.dllist())
    for name in services.IMPLEMENTED_FUNCTIONS
        @test any(method -> method.module === SpaceAGORAMuJoCo, methods(getproperty(services, name)))
    end
    # Contract 2.0.0 order: time before the owner-local slot.
    runtime = SpaceAGORAMuJoCo.ProximitySceneRuntime
    for hook in (services.external_state, services.external_acceleration)
        @test hasmethod(hook, Tuple{runtime, Float64, Int})
        @test !hasmethod(hook, Tuple{runtime, Int, Float64})
    end
end
