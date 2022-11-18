# Inverted Pendulum on a CrazyFlie Quadcopter
 
Members: Anshuk Chigullapalli, Rishi Patel, Harry Zhao, Bella Watters

### Abstract
The goal of this project is to create a controller capable of balancing an inverted pendulum on the Crazyflie quadcopter. This is implemented by utilizing the provided sensor decks and the Motion Capture (MoCap) system present in the lab environment. A pendulum is mounted atop the drone with a hinge that allows for one angular degree of freedom. The equations of motion for the pendulum-quadcopter system are derived and linearized about the equilibrium point where the pendulum is in an upright position. The pose of the pendulum is estimated using the MoCap system and tracking markers on both the pendulum and drone. This pose data is then incorporated into the system observer. During testing, the angular error of the pendulum is analyzed to quantify error in the controller and observer performance. Time permitting, the system will be made independent of the MoCap system.

### Equations of Motion
See dynamics/dynamics.ipynb for the full derivation. The equations of motion use the existing drone kinematics and dynamics derived for AE 483, with the additional kinematics of the pendulum based on the state of the drone. The assumption is made that the pendulum's movement does not affect the drone's motion.

### PyBullet Simulation
The Pybullet setup used in AE 483 was modified to have hinge object and the pendulum object so that the controller designed can best tested in simulation before being implemented on the MoCap system.

### Using the MoCap System
* Running the 
* To receive data from the MoCap system onto your computer, you have to run receive_data.py. This script gets the data from the Mocap system and also parses the marker data to get the angle of the pendulum.
* If you want to get data straight to the drone, you can copy the receive_data.py loop stuff to the receive_data function in flight.py

### Hardware Changes
* The pendulum was made using a 1.6mm diameter 
