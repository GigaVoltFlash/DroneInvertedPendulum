# Inverted Pendulum on a CrazyFlie Quadcopter
 
Members: Anshuk Chigullapalli, Rishi Patel, Harry Zhao, Bella Watters

### Abstract
The goal of this project is to create a controller capable of balancing an inverted pendulum on the Crazyflie quadcopter. This is implemented by utilizing the provided sensor decks and the Motion Capture (MoCap) system present in the lab environment. A pendulum is mounted atop the drone with a hinge that allows for one angular degree of freedom. The equations of motion for the pendulum-quadcopter system are derived and linearized about the equilibrium point where the pendulum is in an upright position. The pose of the pendulum is estimated using the MoCap system and tracking markers on both the pendulum and drone. This pose data is then incorporated into the system observer. During testing, the angular error of the pendulum is analyzed to quantify error in the controller and observer performance. Time permitting, the system will be made independent of the MoCap system.

### Equations of Motion
See dynamics/dynamics.ipynb for the full derivation. The equations of motion use the existing drone kinematics and dynamics derived for AE 483, with the additional kinematics of the pendulum based on the state of the drone. The assumption is made that the pendulum's movement does not affect the drone's motion.

### PyBullet Simulation
The Pybullet setup used in AE 483 was modified to have hinge object and the pendulum object so that the controller designed can best tested in simulation before being implemented on the MoCap system.

### Using the MoCap System

1) Setup up the drone pendulum system to have three reflectors, one at the tip of the drone, one at the pendulum base, and a third along the x-axis. 
2) Connect to the "MECHKNIGHT" Wifi network with password "f33dback5". 
3) Open a terminal/command prompt and retrieve your IPv4 address on the network. It should look like 192.168.1.XXX.
4) Navigate to the "DroneInvertedPendulum" directory on the desktop of the MoCap PC. Open the "motrack_udp_InvertedPendulum.py" script in NotePad++.
5) Change the address in the "address_list" array variable to the IPv4 you found earlier.
6) Save and close the file
7) Open the Motive Application located on the desktop of the MoCap PC. When prompted, Click Open Existing Project and choose the AE483 calibration file.
8) You should now see the three reflectors on the drone in the window. Please do not change anything in this file.
9) Navigate back to the "DroneInvertedPendulum" directory on the desktop of the MoCap PC. Double click the "motrack_udp_InvertedPendulum.py". You should see a lot of data scrolling through the terminal.
10) Now, head to your personal laptop and navigate to the "DroneInvertedPendulum" folder and open "motrack_udp_receive_data.py" in the editor of your choice. 
11) Run the file in terminal. Angle data should now be printing in the terminal. Try moving the pendulum to see the change in angle.
12) The angle output should read about 0 degrees. If this is not what you see, please follow the troubleshooting guide below.

### MoCap Troubleshooting

If the angle being printed into the terminal is not correct, there are several reasons this could be happening. The number 1 reason is the order of the motion capture balls in the code is not the same as the order being sent from the central MoCap Computer. In order to fix this, rest the drone-pendulum system in the center of the room with the pendulum on the negative side of the drone. Head over to the MoCap Computer and verify the visual is the same as real life. On the drone operator's laptop, open "motrack_udp_receive_data.py" in the editor of choice and uncomment lines 47 and 49. When the script is run, the terminal will output the positions of each reflective ball every 4 seconds. As seen from lines 24-26, the order of positions printed should be reference, tip, and base. It is possible to guess and check until the angle printed out is correct, however using the Motive software is much easier. First, select the reference ball, right click and hover over "Marker Information". You will see the position of the ball. Compare this to the output on the terminal and check if it is the first position vector printed. If not, look to lines 29-31, and swap variable names until the the vector of reference is printed first. Repeat these steps for each ball until the positions are printed out in the correct order. Once done, comment lines 47 and 49 and rerun the script. The angle should now be fixed.

All the changes made to the "motrack_udp_receive_data.py" file must be copied to the "optitrack" function in "flightThreaded.py".

The other most common failure point is too many markers in the motion capture zone. Unfortunately, there is not much that can be done besides asking for all markers to be covered or moved out of the zone. 


### Hardware Changes

The pendulum was made using a 1.6mm diameter aluminum rod with a reflective ball at the top and bottom. This was then connected to the drone with a 3D-printed custom hinge design which allowed the pendulum to swing with one degree of freedom. 

<!-- ### File Structure

* client: Contains scripts used for operating the drone and performing analysis on data collected during flight.
* OptiTrack: Contains scripts used for sending and receiving data between the OptiTrack Computer and drone operator's laptop.
* firmware: Contains the firmware code used to fly the drone. -->
 


