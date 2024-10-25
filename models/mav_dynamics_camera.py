"""
mavDynamicsCamera 
    - Add camera dynamics
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
        7/13/2023 - RWB
"""
import numpy as np
from models.mav_dynamics_sensors import MavDynamics as MavDynamicsSensors
import parameters.camera_parameters as CAM
import message_types.msg_delta as MsgDelta
from tools.saturate import saturate

class MavDynamics(MavDynamicsSensors):
    def __init__(self, Ts: float):
        super().__init__(Ts)

###################################
    # public functions
    def update(self, delta: MsgDelta, wind: np.ndarray):
        super().update(delta, wind)
        # gimbal azimuth dynamics
        az = self._state[13,0]
        az_up = az + self._ts_simulation * CAM.az_gain * delta.gimbal_az
        self._state[13,0] = saturate(az_up, -CAM.az_limit, CAM.az_limit)
        # gimbal elevation dynamics
        el = self._state[14,0]
        el_up = el + self._ts_simulation * CAM.el_gain * delta.gimbal_el
        self._state[14,0] = saturate(el_up, -CAM.el_limit, CAM.el_limit)
