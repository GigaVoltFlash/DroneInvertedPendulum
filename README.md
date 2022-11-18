# Inverted Pendulum on a CrazyFlie Quadcopter
 
Members: Anshuk Chigullapalli, Rishi Patel, Harry Zhao, Bella Watters

### Abstract
The goal of this project is to create a controller capable of balancing an inverted pendulum on the Crazyflie quadcopter. This is implemented by utilizing the provided sensor decks and the Motion Capture (MoCap) system present in the lab environment. A pendulum is mounted atop the drone with a hinge that allows for one angular degree of freedom. The equations of motion for the pendulum-quadcopter system are derived and linearized about the equilibrium point where the pendulum is in an upright position. The pose of the pendulum is estimated using the MoCap system and tracking markers on both the pendulum and drone. This pose data is then incorporated into the system observer. During testing, the angular error of the pendulum is analyzed to quantify error in the controller and observer performance. Time permitting, the system will be made independent of the MoCap system.

### Equations of Motion
See dynamics/dynamics.ipynb for the full derivation. The equations of motion use the existing drone kinematics and dynamics derived for AE 483, with the additional kinematics of the pendulum based on the state of the drone. The assumption is made that the pendulum's movement does not affect the drone's motion.

### PyBullet Simulation
The Pybullet setup used in AE 483 was modified to have hinge object and the pendulum object so that the controller designed can best tested in simulation before being implemented on the MoCap system.

### Using the MoCap System

* 1. ) Setup up the drone pendulum system to have three reflectors, one at the tip of the drone, one at the pendulum base, and a third along the x-axis. 
* 2. ) Connect to the "MECHKNIGHT" Wifi network with password "f33dback5". 
* 3. ) Open a terminal/command prompt and retreive your IPv4 address on the network. It should look like 192.168.1.XXX.
* 4. ) Navigate to the "DroneInvertedPendulum" directory on the desktop of the MoCap PC. Open the "motrack_udp_InvertedPendulum.py" script in NotePad++.
* 5. ) Change the address in the "address_list" array variable to the IPv4 you found earlier.
* 6. ) Save and close the file
* 7. ) Open the Motive Applicaton located on the desktop of the MoCap PC. When prompted, Click Open Existing Project and choose the AE483 calibration file.
* 8. ) You should now see the three reflectors on the drone in the window. Please do not change anything in this file.
* 10. ) Navigate back to the DroneInvertedPendulum" directory on the desktop of the MoCap PC. Double click the "motrack_udp_InvertedPendulum.py". You should see a lot of data scrolling through the terminal.
* 11. ) Now, head to your personal laptop and navigate to the "DroneInvertedPendulum" folder and open receiveData.py in the editor of your choice. You should not have to change anything in here unless you need to test math stuff. 
* 12. ) Run the receiveData.py in terminal. You should immediately see a lot of data scrolling through the window. If you do not, follow the steps in the trouble shooting section below.
* 13. ) If you made it here, your laptop is setup to receive data from the MoCap system. 

### MoCap Troubleshooting

### Hardware Changes
* The pendulum was made using a 1.6mm diameter 
