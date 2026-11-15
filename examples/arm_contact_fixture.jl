using SpaceAGORA, SpaceAGORAMuJoCo, StaticArrays, LinearAlgebra

"""A floating bus with three compliant arm links. A box stop is fixed to the
bus beyond the arm tip. Initial rigid rotation stretches the compliant chain
toward the stop, without external loading or a prescribed kinematic base.
"""
function arm_contact_configuration(;duration=.5,h=2.0^-12,friction=.4,gap=.005,spin=.8,
        tolerance=1e-10,origin=SVector(7e6,0.,0.),simultaneous=false,exact_sticking=false,capture_speed=1e-7)
    SM=SpaceAGORA.SimulationModel
    bus=Link(root=true,m=20.,dims=MVector(1.,.8,.6))
    arm=SM.default_robot_arm_model(link_lengths_m=(.3,.25,.2),link_radii_m=(.03,.025,.02),
        link_masses_kg=(.7,.5,.3),mount_offset_body=(.5,0.,0.))
    plan=SM.RobotArmPlanBuffer(arm,SM.RobotArmBasePose(SVector(0.,0.,0.)),zeros(3)).plan
    effector=SM.RobotArmReactionEffector(plan=plan,k_translation_n_m=200.,
        c_translation_n_s_m=.2,k_rotation_n_m_rad=2.,c_rotation_n_m_s_rad=.02)
    sc=SpacecraftModel(links=[bus],root=bus,prop_mass=0.,inertia_tensor=bus.inertia,
        initial_condition=CartesianInitialCondition(origin,SVector(0.,0.,0.);ang_vel=SVector(0.,0.,spin)))
    planet=make_no_gram_planet(:earth)
    args=SimulationConfiguration(simulation_settings=SimulationSettings(verbose=false,results=false),
        mission_configuration=MissionConfiguration(mission_type=MissionTime,mission_time=duration,
            orientation_sim=true,keplerian=false,data_rate=2.0^-11),
        environment_model=EnvironmentModel(EI=120.,planet=planet,
            thermal_model=MaxwellianHeat(thermal_accomodation_factor=1.,planet=planet),
            density_model=NoAtmosphereModel(),ephemerides_model=SimpleEphemeridesModel()),
        dynamics_model=DynamicsModel([sc],(effector,)),initial_time=InitialTime(year=2020,month=1,day=1),
        solver_config=SolverConfig(solver_mode=:tsit5),
        integration_tolerances=IntegrationTolerances(dt_max_orbit=h,dt_max_atmosphere=h,
            reltol_orbit=tolerance,abstol_orbit=tolerance/10,reltol_quaternion=tolerance,abstol_quaternion=tolerance/10,
            reltol_angular_rate=tolerance,abstol_angular_rate=tolerance/10))
    # Native capsules point along local z; rotate them onto the arm's local x.
    q=SVector(0.,sin(pi/4),0.,cos(pi/4))
    geoms=(ContactGeometry(1,:box,(.05,.2,.2);position=(1.32+gap,0.,0.)),
        (ContactGeometry(i+1,:capsule,(link.radius_m,norm(link.vector_parent)/2);quaternion=q)
            for (i,link) in enumerate(arm.links))...)
    pairs=((1,4),)
    if simultaneous
        geoms=(geoms...,ContactGeometry(1,:box,(.05,.2,.2);position=(1.125+.6gap,0.,0.)))
        pairs=(pairs...,(5,3))
    end
    contact=exact_sticking ? CoulombContact(geoms,pairs;friction,capture_speed) : PrimitiveContact(geoms,pairs;friction)
    return (;args,contact,plan)
end
