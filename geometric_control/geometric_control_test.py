import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import matplotlib.pyplot as plt
from tools.rotations import euler_to_rotation, rotation_to_euler, euler_to_quaternion
from geometric_control.geometric_controller import GeometricController

# trajectories
from planners.trajectorygenerator.scripts.trajectory import Trajectory
from planners.trajectorygenerator.scripts.sinusoidal_trajectory_member import SinusoidalTrajectoryMember
from planners.trajectorygenerator.scripts.linear_trajectory_member import LinearTrajectoryMember
import planners.trajectorygenerator.scripts.trajectory_plotter as trajectory_plotter

def main():
    # 5m radius slanted orbit
    # centered at 0, -5
    # yaw pointing along trajectory
    circle_Pn_traj_memb = SinusoidalTrajectoryMember(0., 5., 30., 0.)
    circle_Pe_traj_memb = SinusoidalTrajectoryMember(-5., 5., 30., np.pi/2)
    circle_Pd_traj_memb = SinusoidalTrajectoryMember(-10., 5., 30., np.pi/2) 
    circle_Ppsi_traj_memb = LinearTrajectoryMember(0.0, -2*np.pi/30.)

    circle_trajectory = Trajectory(
        [circle_Pn_traj_memb,
            circle_Pe_traj_memb,
            circle_Pd_traj_memb,
            circle_Ppsi_traj_memb],
        time_scale = .3
    )

    t_end = 100
    trajectory_plotter.plotVsTime(circle_trajectory, 0, 0., t_end)
    trajectory_plotter.plot3D(circle_trajectory, 0, [0, 1, 2], 0., t_end)

    t = np.arange(0., t_end, .01)

    F = np.zeros((2, len(t)))
    euler = np.zeros((3, len(t)))
    omega = np.zeros((3, len(t)))

    gc = GeometricController()

    for i in range(len(t)):
        trajectory_flag_t = circle_trajectory.evalUpToKr(t[i], 4)

        pd_d0 = trajectory_flag_t[0:3,0]
        pd_d1 = trajectory_flag_t[0:3,1]
        psi_d0 = trajectory_flag_t[3,0]

        q_b2i = euler_to_quaternion(0., 0., psi_d0)
        pd_d1_body = euler_to_rotation(0., 0., psi_d0).T @ pd_d1
        
        state = np.concatenate([pd_d0, pd_d1_body, q_b2i.reshape(-1)])
        Fi, Ri, omegai, pd, vd_b_pitch = gc.update(state, trajectory_flag_t)

        F[:,i] = Fi

        phi, theta, psi = rotation_to_euler(Ri)

        print("t = {}, \nR = {}".format(t[i], Ri))

        euler[:,i] = np.array([phi, theta, psi])

        omega[:,i] = omegai

    fig, ax = plt.subplots(2,1)
    fig.suptitle("Desired Thrust")
    ax[0].plot(t, F[0,:], label="T_x")
    ax[0].legend()
    # ax[0].set_ylim([-1., 10.])
    ax[1].plot(t, F[1,:], label="T_z")
    ax[1].legend()
    # ax[1].set_ylim([-10., 1.])

    fig, ax = plt.subplots(3,1)
    fig.suptitle("Angles")
    ax[0].plot(t, euler[0,:], label="phi")
    ax[0].legend()
    ax[1].plot(t, euler[1,:], label="theta")
    ax[1].legend()
    ax[2].plot(t, euler[2,:], label="psi")
    ax[2].legend()

    fig, ax = plt.subplots(3,1)
    fig.suptitle("Rates")
    ax[0].plot(t, omega[0,:], label="omega_x")
    ax[0].legend()
    ax[1].plot(t, omega[1,:], label="omega_y")
    ax[1].legend()
    ax[2].plot(t, omega[2,:], label="omega_z")
    ax[2].legend()

    plt.show()

main()