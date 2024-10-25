"""
mavsim_python
    - Chapter 13 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        4/1/2022 - RWB
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
from models.wind_simulation import WindSimulation
from models.target_dynamics import TargetDynamics
from models.mav_dynamics_camera import MavDynamics
from models.camera import Camera
from models.gimbal import Gimbal
from controllers.autopilot import Autopilot
from estimators.observer import Observer
from estimators.geolocation import Geolocation
from planners.path_follower import PathFollower
from planners.path_manager_follow_target import PathManager
from message_types.msg_world_map import MsgWorldMap
from message_types.msg_waypoints import MsgWaypoints
from viewers.view_manager import ViewManager

# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
wind = WindSimulation(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
observer = Observer(SIM.ts_simulation)
gimbal = Gimbal()
camera = Camera()
path_follower = PathFollower()
path_manager = PathManager()
geolocation = Geolocation(SIM.ts_simulation)
waypoints = MsgWaypoints()
world_map = MsgWorldMap(height=0.)
target = TargetDynamics(SIM.ts_simulation, world_map)
#quitter = QuitListener()
viewers = ViewManager(camera=True,
                      geo=True,
                      )

# initialize the simulation time
sim_time = SIM.start_time
end_time = 200

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------observer-------------
    measurements = mav.sensors()  # get sensor measurements
    #estimated_state = observer.update(measurements)  
    estimated_state = mav.true_state
    camera.updateProjectedPoints(mav.true_state, target.position())
    pixels = camera.getPixels()
    estimated_target_position = geolocation.update(estimated_state, pixels)
    #estimated_target_position=target.position()

    # -------path manager-------------
    path = path_manager.update(target.position())
    #path = path_manager.update(estimated_target_position)

    # -------path follower-------------
    autopilot_commands = path_follower.update(path, estimated_state)

    # -------autopilot-------------
    delta, commanded_state = autopilot.update(autopilot_commands, estimated_state)
    gimbal_cmd = gimbal.pointAtPosition(estimated_state, target.position())  
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
        estimated_target=estimated_target_position,
    )

    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation

# close viewers
viewers.close(dataplot_name="ch13_data_plot")




