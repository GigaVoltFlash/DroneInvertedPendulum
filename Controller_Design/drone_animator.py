import pandas as pd
import os
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import math
'''
RUN THIS FILE AFTER 'rk4_planar_sim.ipynb' to see animation of drone in 2D
'''

l_pen = 150e-3 # (m)
drone_w = 0.3
drone_h = 0.02

def animator (times, state_data) :
    
    plt.style.use('dark_background')
    # fig = plt.figure()
    fig, (drone_ani) = plt.subplots(1)
    drone_ani.set_title("Drone Balancing Inverted Pendulum in 2D SIM", color="orange")
    drone_ani.set_xlabel('x drone displacement (m)')
    drone_ani.set_ylabel('height agl (m)')

    # x_plot.set_title("X-Position from Origin", color="blue")
    # x_plot.set_xlabel('time (s)')
    # x_plot.set_ylabel('x-position (m)')
    # fig.tight_layout()

    def get_pend_cordinates(alpha):
        """Return the (x, y) coordinates of the top of the pendulum at angle alpha."""
        return l_pen * np.sin(alpha), l_pen * np.cos(alpha)
    
    ### Drone Animation Setup
    x0, y0 = get_pend_cordinates(0)
    pendulum, = drone_ani.plot([0, x0], [0, y0], lw=3, c='orange')
    box = patches.Rectangle((-0.5*drone_w, -drone_h), drone_w, drone_h, fc='w')

    ## X-Plot Animation Plot Setup
    # line, = x_plot.plot(0, 0)
    

    def rotate(origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = point

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy

    def init():
        # Keep Aspect Ratio a Square to avoid distortion
        drone_ani.set_xlim(-1.5, 3.0)
        drone_ani.set_ylim(1.5, 6.)
        drone_ani.add_patch(box)

    def animate(i):
        # Animation updated at frame i
        o_x, o_y = state_data[0][i], state_data[1][i]
        x, y = get_pend_cordinates(state_data[3][i])
        pendulum.set_data([o_x, o_x + x], [o_y, o_y + y])
        
        box.set_angle(-np.rad2deg(state_data[2][i]))
        box.set_xy(rotate((o_x, o_y), (o_x-(0.5*drone_w), o_y-drone_h), -state_data[2][i]))




    # Run the Animation
    nframes = len(times)
    dt = 0.01
    intvl = dt * 1000
    ani = animation.FuncAnimation(fig, animate, frames=nframes, init_func = init, repeat=True,
                                interval=intvl)
    plt.show()

if __name__ == "__main__":
    times = []
    state_data = [[],[],[],[]]
    output_file = os.path.join(os.path.dirname(__file__), '../Controller_Design/sim.csv')
    file_data = pd.read_csv(output_file)
    times = file_data["time"].values
    state_data[0] = file_data["x"].values
    state_data[1] = file_data["z"].values
    state_data[2] = file_data["theta"].values
    state_data[3] = file_data["alpha"].values

    animator(times, state_data)