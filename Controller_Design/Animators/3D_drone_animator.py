from util.basic import Sphere, Line, Arrow
import numpy as np
import pandas as pd

class Uav:
    L = 0.32 # length of pendulum (m)

    '''
    Draws a quadrotor at a given position, with a given attitude.
    '''

    def __init__(self, ax, arm_length):
        '''
        Initialize the quadrotr plotting parameters.

        Params:
            ax: (matplotlib axis) the axis where the sphere should be drawn
            arm_length: (float) length of the quadrotor arm

        Returns:
            None
        '''

        self.ax = ax
        self.arm_length = arm_length

        self.b1 = np.array([1.0, 0.0, 0.0]).T
        self.b2 = np.array([0.0, 1.0, 0.0]).T
        self.b3 = np.array([0.0, 0.0, 1.0]).T

        # Center of the quadrotor
        self.body = Sphere(self.ax, 0.08, 'y')

        # Each motor
        self.motor1 = Sphere(self.ax, 0.05, 'r')
        self.motor2 = Sphere(self.ax, 0.05, 'g')
        self.motor3 = Sphere(self.ax, 0.05, 'b')
        self.motor4 = Sphere(self.ax, 0.05, 'b')

        # Arrows for the each body axis
        self.arrow_b1 = Arrow(ax, self.b1, 'r')
        self.arrow_b2 = Arrow(ax, self.b2, 'g')
        self.arrow_b3 = Arrow(ax, self.b3, 'b')

        # Quadrotor arms
        self.arm_b1 = Line(ax)
        self.arm_b2 = Line(ax)

        # Pendulum arm
        self.arm_b3 = Line(ax)
    

    def draw_at(self, x=np.array([0.0, 0.0, 0.0]).T, R=np.eye(3), pend_pos = np.zeros(2)):
        '''
        Draw the quadrotor at a given position, with a given direction

        Args:
            x: (3x1 numpy.ndarray) position of the center of the quadrotor, 
                default = [0.0, 0.0, 0.0]
            R: (3x3 numpy.ndarray) attitude of the quadrotor in SO(3)
                default = eye(3)
        
        Returns:
            None
        '''

        #pendulum setup
        r = pend_pos[0]
        s = pend_pos[1]

        # First, clear the axis of all the previous plots
        self.ax.clear()

        # Center of the quadrotor
        self.body.draw_at(x)

        # Each motor
        self.motor1.draw_at(x + R.dot(self.b1) * self.arm_length)
        self.motor2.draw_at(x + R.dot(self.b2) * self.arm_length)
        self.motor3.draw_at(x + R.dot(-self.b1) * self.arm_length)
        self.motor4.draw_at(x + R.dot(-self.b2) * self.arm_length)

        # Arrows for the each body axis
        self.arrow_b1.draw_from_to(x, R.dot(self.b1) * self.arm_length * 1.8)
        self.arrow_b2.draw_from_to(x, R.dot(self.b2) * self.arm_length * 1.8)
        # self.arrow_b3.draw_from_to(x, R.dot(self.b3) * self.arm_length * 1.8)

        # Quadrotor arms
        self.arm_b1.draw_from_to(x, x + R.dot(-self.b1) * self.arm_length)
        self.arm_b2.draw_from_to(x, x + R.dot(-self.b2) * self.arm_length)
        
        #Pendulum
        # self.arm_b3.draw_from_to(x, x + R.dot(self.b3) * 0.5) # 0.5 is pendulum length
        # self.arm_b3.draw_from_to(x, x * 0.5 + 1) # 0.5 is pendulum length
        self.arm_b3.draw_from_to(x, x + [r, s, (self.L**2 + r**2 + s**2)**(0.5)])



if __name__ == '__main__':
    from util.utils import ypr_to_R

    from matplotlib import animation
    from mpl_toolkits.mplot3d import Axes3D

    import matplotlib.pyplot as plt

    
    def update_plot(i, x, R, pend_pos):
        uav_plot.draw_at(x[:, i], R[:, :, i], pend_pos[:,i])
        
        # These limits must be set manually since we use
        # a different axis frame configuration than the
        # one matplotlib uses.
        xmin, xmax = -0.5,0.5
        ymin, ymax = -0.5,0.5
        zmin, zmax = 1,0

        ax.set_xlim([xmin, xmax])
        ax.set_ylim([ymax, ymin])
        ax.set_zlim([zmax, zmin])

    # Initiate the plot
    plt.style.use('seaborn')

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    arm_length = 0.24  # in meters
    uav_plot = Uav(ax, arm_length)


    # Load in 3-D Sim Data

    # state: [time,x,y,z,yaw,pitch,roll,r,s]
    times = []
    state_data = [[],[],[],[],[],[],[],[]]
    output_file = '../DroneInvertedPendulum/Controller_Design/3d-sim.csv'
    file_data = pd.read_csv(output_file)
    times = file_data["time"].values
    state_data[0] = file_data["x"].values
    state_data[1] = file_data["y"].values
    state_data[2] = file_data["z"].values
    state_data[3] = file_data["yaw"].values
    state_data[4] = file_data["pitch"].values
    state_data[5] = file_data["roll"].values
    state_data[6] = file_data["r"].values
    state_data[7] = file_data["s"].values
    nframes = len(times)
    x = np.array([state_data[0],state_data[1],state_data[2]])
    pend_pos = np.array([state_data[6], state_data[7]])

    #Populate Rotation Matrix
    R = np.zeros((3, 3, nframes))
    for i in range(nframes):
        ypr = np.array([state_data[3][i], state_data[4][i], state_data[5][i]])
        R[:, :, i] = ypr_to_R(ypr, degrees=False)

    intvl = 1 # milliseconds

    # Run the simulation
    ani = animation.FuncAnimation(fig, update_plot, frames=nframes,interval=intvl, \
        fargs=(x, R,pend_pos))
    # ani.save('test.gif', writer='imagemagick', fps=60)
    plt.show()