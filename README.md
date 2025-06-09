This is the control code for the drone using the IMU device.



*****ControlCodeDrone - Code to control the real drone*****

The snapki adress and desired characteristic is already defined in the code but the Drone URI must be defined by the user depending on the drone.
It is necessary to have bluetooth turned on and the radio antena from crazyflie connected to the PC with all drivers installed.



*****ControloSimulação - Code to control the simulated drone*****

To control the drone in webots simulation first it is necessary to replace the crazyflie_py_wallfollowing.py file in the folder referent to the bitcraze controllers by the one provided. Next to run the simulation it is necessary to define the crazyflie_py_wallfollowing as the controller of the drone in the scene tree side bar. After that, the user should run the ControloSimulação.py script to control the drone.
