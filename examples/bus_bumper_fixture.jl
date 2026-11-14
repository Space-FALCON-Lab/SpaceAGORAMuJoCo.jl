# Shared by the reproducible example and the required paired-core tests.
using SpaceAGORA, StaticArrays, LinearAlgebra

function bus_bumper_configuration(; h=2.0^-12, duration=2., output_step=2.0^-11,
        offset=0., tangent=0., origin=SVector(7e6,0.,0.), boost=SVector(0.,0.,0.))
    bus=Link(root=true,m=20.,dims=MVector(1.,.8,.6))
    relative_velocity=SVector(-.25,tangent,0.)
    node=CompliantTopologyNode(:bumper; mass_kg=1.,inertia_body_kg_m2=.004,
        position=SVector(.11,0.,0.),velocity=relative_velocity)
    edge=CompliantTopologyEdge(:mount,0,1; parent_point_body=SVector(0.,0.,0.),
        child_point_body=SVector(-.11,0.,0.),k_translation_n_m=200.,
        c_translation_n_s_m=0.,k_rotation_n_m_rad=2.,c_rotation_n_m_s_rad=.02)
    attachment=CompliantAttachment(model=build_compliant_topology([node],[edge]),
        link=bus,mount_point=SVector(.5,offset,0.))
    sc=SpacecraftModel(links=[bus],root=bus,prop_mass=0.,inertia_tensor=bus.inertia,
        initial_condition=CartesianInitialCondition(origin,boost-relative_velocity/21),attachments=[attachment])
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
            reltol_orbit=1e-10,abstol_orbit=1e-11,reltol_quaternion=1e-10,abstol_quaternion=1e-11,
            reltol_angular_rate=1e-10,abstol_angular_rate=1e-11))
end
