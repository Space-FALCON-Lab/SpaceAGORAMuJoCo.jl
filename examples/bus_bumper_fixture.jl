# Shared by the reproducible example and the required paired-core tests.
using SpaceAGORA, StaticArrays, LinearAlgebra

function bus_bumper_configuration(; h=2.0^-12, duration=2., output_step=2.0^-11,
        offset=0., tangent=0., origin=SVector(7e6,0.,0.), boost=SVector(0.,0.,0.),
        attitude=SVector(0.,0.,0.,1.), gap=.01, normal_speed=.25,
        translation_damping=0., tolerance=1e-10)
    bus=Link(root=true,m=20.,dims=MVector(1.,.8,.6))
    q=normalize(SVector{4,Float64}(attitude))
    x,y,z,w=q
    R=@SMatrix [1-2(y*y+z*z) 2(x*y-z*w) 2(x*z+y*w);
        2(x*y+z*w) 1-2(x*x+z*z) 2(y*z-x*w);
        2(x*z-y*w) 2(y*z+x*w) 1-2(x*x+y*y)]
    relative_velocity=SVector(-normal_speed,tangent,0.)
    node=CompliantTopologyNode(:bumper; mass_kg=1.,inertia_body_kg_m2=.004,
        position=SVector(.1+gap,0.,0.),velocity=relative_velocity)
    edge=CompliantTopologyEdge(:mount,0,1; parent_point_body=SVector(0.,0.,0.),
        child_point_body=SVector(-.11,0.,0.),k_translation_n_m=200.,
        c_translation_n_s_m=translation_damping,k_rotation_n_m_rad=2.,c_rotation_n_m_s_rad=.02)
    attachment=CompliantAttachment(model=build_compliant_topology([node],[edge]),
        link=bus,mount_point=SVector(.5,offset,0.))
    sc=SpacecraftModel(links=[bus],root=bus,prop_mass=0.,inertia_tensor=bus.inertia,
        initial_condition=CartesianInitialCondition(origin,boost-R*relative_velocity/21;q=q),attachments=[attachment])
    planet=make_no_gram_planet(:earth)
    return SimulationConfiguration(
        simulation_settings=SimulationSettings(verbose=false,results=false),
        mission_configuration=MissionConfiguration(mission_type=MissionTime,mission_time=duration,
            orientation_sim=true,keplerian=false,data_rate=output_step),
        environment_model=EnvironmentModel(EI=120.,planet=planet,
            thermal_model=MaxwellianHeat(thermal_accomodation_factor=1.,planet=planet),density_model=NoAtmosphereModel(),ephemerides_model=SimpleEphemeridesModel()),
        dynamics_model=DynamicsModel([sc],()),initial_time=InitialTime(year=2020,month=1,day=1),
        solver_config=SolverConfig(solver_mode=:tsit5),
        integration_tolerances=IntegrationTolerances(dt_max_orbit=h,dt_max_atmosphere=h,
            reltol_orbit=tolerance,abstol_orbit=tolerance/10,reltol_quaternion=tolerance,abstol_quaternion=tolerance/10,
            reltol_angular_rate=tolerance,abstol_angular_rate=tolerance/10))
end
