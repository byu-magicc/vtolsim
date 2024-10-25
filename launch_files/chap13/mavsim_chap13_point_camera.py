"""
mavsim_python
    - Chapter 13 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        3/30/2022 - RWB
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
        4/12/2024 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from tools.quit_listener import QuitListener
import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN
from models.mav_dynamics_camera import MavDynamics
from models.wind_simulation import WindSimulation
from models.camera import Camera
from models.target_dynamics import TargetDynamics
from models.gimbal import Gimbal
from controllers.autopilot import Autopilot
from estimators.observer import Observer
#from estimators.observer_full import Observer
from estimators.geolocation import Geolocation
from planners.path_planner import PathPlanner
from planners.path_follower import PathFollower
from planners.path_manager import PathManager
from viewers.view_manager import ViewManager
from message_types.msg_world_map import MsgWorldMap
#quitter = QuitListener()

# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
wind = WindSimulation(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
observer = Observer(SIM.ts_simulation)
gimbal = Gimbal()
camera = Camera()
path_follower = PathFollower()
path_manager = PathManager()
path_planner = PathPlanner(type='rrt_dubins')
world_map = MsgWorldMap()
target = TargetDynamics(SIM.ts_simulation, world_map)
viewers = ViewManager(camera=True)

# initialize the simulation time
sim_time = 0.
end_time = 200.

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------observer-------------
    measurements = mav.sensors()  # get sensor measurements
    #estimated_state = observer.update(measurements)  # estimate states from measurements
    estimated_state = mav.true_state
    camera.updateProjectedPoints(mav.true_state, target.position())

    # -------path planner - ----
    if path_manager.manager_requests_waypoints is True:
        waypoints = path_planner.update(world_map, estimated_state, PLAN.R_min)

    # -------path manager-------------
    path = path_manager.update(waypoints, estimated_state, PLAN.R_min)

    # -------path follower-------------
    autopilot_commands = path_follower.update(path, estimated_state)

    # -------autopilot-------------
    delta, commanded_state = autopilot.update(autopilot_commands, estimated_state)
    gimbal_cmd = gimbal.pointAtPosition(estimated_state, target.position()) 
    #gimbal_cmd = gimbal.pointAtGround(estimated_state) 
        # point gimbal at target position
    delta.gimbal_az = gimbal_cmd.item(0)
    delta.gimbal_el = gimbal_cmd.item(1)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics
    target.update()  # propagate the target dynamics

    # -------update viewer-------------
    viewers.update(
        sim_time,
        true_state=mav.true_state,  # true states
        estimated_state=estimated_state,  # estimated states
        commanded_state=commanded_state,  # commanded states
        delta=delta,  # inputs to aircraft
        path=path, # path
        waypoints=waypoints, # waypoints
        map=world_map,  # map of world
        target=target.position(), # position of target
        camera=camera, # camera pixels
    )

    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation

# close viewers
viewers.close(dataplot_name="ch13_data_plot")




