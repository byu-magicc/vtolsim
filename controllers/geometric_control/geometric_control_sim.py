"""
vtolsim
    - Update history:
        5/8/2019 - R.W. Beard
        3/12/2024 - RWB        
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import parameters.simulation_parameters as SIM
# import parameters.spline_parameters as SPLP
#from message_types.msg_convert import *
from models.vtol_dynamics import VtolDynamics
# from controllers.lqr.lqr_control import LqrControl
# from controllers.low_level_control import LowLevelControl
# from planners.trajectory.spline_trajectory import SplineTrajectory
# from planners.trajectory.differential_flatness import DifferentialFlatness
from viewers.view_manager import ViewManager
# from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState

# import time
# from message_types.msg_controls import MsgControls
from _control_allocation_old.nonlinear_control_allocation import NonlinearControlAllocation
from controllers.rate_control import RateControl
from geometric_control.geometric_controller import GeometricController
#from geometric_control.geometric_zthrust_controller import GeometricZThrustController
#from tools.msg_convert import *
#from tools.rotations import rotation_to_quaternion

# trajectories
from planners.trajectorygenerator.scripts.trajectory import Trajectory
from planners.trajectorygenerator.scripts.sinusoidal_trajectory_member import SinusoidalTrajectoryMember
from planners.trajectorygenerator.scripts.linear_trajectory_member import LinearTrajectoryMember
from planners.trajectorygenerator.scripts.quadratic_trajectory_member import QuadraticTrajectoryMember
from trajectory_planning.takeoff_coast_land_trajectory import TakeoffCoastLandTrajectory
from planners.trajectorygenerator.scripts.setup_polynomial_trajectory import setup_polynomial_acc_2021_tcl_trajectory, setup_polynomial_airsim_demo_tcl_trajectory, setup_bspline_airsim_demo_trajectory
import planners.trajectorygenerator.scripts.trajectory_plotter

# initialize elements of the architecture
wind = np.array([[0., 0., 0., 0., 0., 0.]]).T
vtol = VtolDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)

#np.set_printoptions(precision=4, linewidth=200, suppress=True)

# INITIALIZE TRAJECTORIES

# 5m radius circular orbit in i-j plane at 10m alt.
# centered at 0, -5
# yaw pointing along trajectory
circle_Pn_traj_memb = SinusoidalTrajectoryMember(0., 5., 30., 0.)
circle_Pe_traj_memb = SinusoidalTrajectoryMember(-5., 5., 30., np.pi/2)
circle_Pd_traj_memb = SinusoidalTrajectoryMember(-2., 2.0, 30., np.pi/2)
circle_Psi_traj_memb = LinearTrajectoryMember(0.0, -2*np.pi/30.)

circle_trajectory = Trajectory(
    [circle_Pn_traj_memb,
        circle_Pe_traj_memb,
        circle_Pd_traj_memb,
        circle_Psi_traj_memb],
    time_scale = 1.
)

pringle_Pn_traj_memb = SinusoidalTrajectoryMember(0., 5., 30., 0.)
pringle_Pe_traj_memb = SinusoidalTrajectoryMember(-5., 5., 15., 0.)
pringle_Pd_traj_memb = SinusoidalTrajectoryMember(10., 5.0, 30., np.pi/2)
pringle_Psi_traj_memb = LinearTrajectoryMember(0.0, -2*np.pi/30.)

pringle_trajectory = Trajectory(
    [pringle_Pn_traj_memb,
        pringle_Pe_traj_memb,
        pringle_Pd_traj_memb,
        pringle_Psi_traj_memb],
    time_scale = .5
)

line_Pn_traj_memb = QuadraticTrajectoryMember(.4, 0., 0.) # Pn = .5*.5*t**2
line_Pe_traj_memb = LinearTrajectoryMember(0.0, 0.0)
line_Pd_traj_memb = LinearTrajectoryMember(0.0, 0.)
line_Psi_traj_memb = LinearTrajectoryMember(0.0, 0.0)

line_trajectory = Trajectory(
    [line_Pn_traj_memb,
        line_Pe_traj_memb,
        line_Pd_traj_memb,
        line_Psi_traj_memb],
)

tcl_trajectory = TakeoffCoastLandTrajectory(
    10, # t_takeoff
    10, # t_coast
    15, # t_land
    10, # max_altitude
    5) # max_horizontal_velocity

acc_poly_tcl_trajectory = setup_polynomial_acc_2021_tcl_trajectory(.2, optimize_segment_times=False)
airsim_poly_tcl_trajectory_nopt = setup_polynomial_airsim_demo_tcl_trajectory(.2, optimize_segment_times=False)
airsim_poly_tcl_trajectory_opt = setup_polynomial_airsim_demo_tcl_trajectory(.2, optimize_segment_times=True)
airsim_bspline_tcl_trajectory = setup_bspline_airsim_demo_trajectory(.4)
bspline_time_scale = airsim_bspline_tcl_trajectory.members[0].total_time*.2/airsim_poly_tcl_trajectory_opt.members[0].total_time
airsim_bspline_tcl_trajectory.time_scale = bspline_time_scale


## ---------------------------------
#  Edit here to change the trajectory that gets used

# sim_trajectory = circle_trajectory; traj_name = "Circle"; SIM.end_time = 30.
# sim_trajectory = pringle_trajectory; traj_name = "Pringle"
# sim_trajectory = line_trajectory; traj_name = "Line"
# sim_trajectory = tcl_trajectory; traj_name = "TCL"; SIM.end_time = tcl_trajectory.t6
sim_trajectory = acc_poly_tcl_trajectory; traj_name = "ACCPolyTCL"; SIM.end_time = acc_poly_tcl_trajectory.members[0].num_segments/acc_poly_tcl_trajectory.time_scale  + 5
# sim_trajectory = airsim_poly_tcl_trajectory_opt; traj_name = "AirSimPolyTCLOpt"; SIM.end_time = airsim_poly_tcl_trajectory_opt.members[0].num_segments/airsim_poly_tcl_trajectory_opt.time_scale  + 5
# sim_trajectory = airsim_poly_tcl_trajectory_nopt; traj_name = "AirSimPolyTCLNOpt"; SIM.end_time = airsim_poly_tcl_trajectory_nopt.members[0].num_segments/airsim_poly_tcl_trajectory_nopt.time_scale  + 5
# sim_trajectory = airsim_bspline_tcl_trajectory; traj_name = "AirSimBSplineTCL"; SIM.end_time = airsim_bspline_tcl_trajectory.members[0].total_time/airsim_bspline_tcl_trajectory.time_scale  + 5

## ---------------------------------

# draw the trajectory
npts = int((SIM.end_time - SIM.start_time)/.01)
trajectory_position_points, _ = trajectory_plotter.evalVector(sim_trajectory, 0, SIM.start_time, SIM.end_time, npts)

vtol_view.addTrajectory(trajectory_position_points[:3,:])

# initialize geometric controller
geom_ctrl = GeometricController(
    time_step = SIM.ts_control, 
    seed_optimizer=True, 
    constrain_pitch_rate=True)
    # geom_ctrl = GeometricZThrustController(time_step = SIM.ts_control, seed_optimizer=True, constrain_pitch_rate=True)

#initialize low level control
rate_control = RateControl(ts_control=SIM.ts_control)
control_alloc = NonlinearControlAllocation()

# initialize the simulation time
sim_time = SIM.start_time
Ts = SIM.ts_simulation

# main simulation loop
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol._state  # estimated state is current state

    # ------ Trajectory follower
    traj_derivatives_at_t = sim_trajectory.evalUpToKr(sim_time, 4)
    # traj_derivatives_at_t = np.zeros(16).reshape((4,4))

    #------- High Level controller-------------
    T, R_d, omega_c, pd_i, vd_b = geom_ctrl.update(estimated_state[0:10], traj_derivatives_at_t)

    #------- Low Level Controller -------------
    omega = estimated_state[10:13,0]
    tau_c = rate_control.update(omega_c, omega)
    delta = control_alloc.update(T, tau_c, estimated_state, vtol._Va)
    # delta = control_alloc.update_lp(T, tau_c, vtol._Va)

    #-------update physical system-------------
    vtol.update(delta, wind)  # propagate the MAV dynamics

    #-------update viewers-------------
    commanded_state = MsgState(
        pos=pd_i,
        vel=vd_b,
        R=R_d,
        omega=omega_c,
        )
    # thrust_torque_d = np.concatenate([T, tau_c]).reshape((-1,1))
    # thrust_torque = np.concatenate([np.copy(vtol.total_thrust[[0,2]]), np.copy(vtol.total_torque)])

    #-------update viewers-------------
    viewers.update(
        sim_time,
        vtol.true_state,  # true states
        estimated_state,  # estimated states
        commanded_state,  # commanded states
        delta,  # inputs to aircraft
        None,  # measurements
    )
    #-------increment time-------------
    sim_time += SIM.ts_simulation
print("Done with simulation")
