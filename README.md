# lidar-scanner-final
basic lidar scanner using an msp432e401y, a single ToF sensor &amp; a stepper motor. MATLAB for serial comms &amp; data processing &lt;3

it does work, i promise. check hallway_test & ye_olde_hallway.

though you probably will have to heavily edit the pin assignments in the C code if using on any other microcontroller. also the serial communication line for any other computer.

pls note (for my own reference tbh):
the separation between scanned layers is in the matlab code and is by default set to the number of layers. it can be changed ezpz
90% sure the button to start the program is pj1 (onboard button) which can also be swapped out to a button on a breadboard if pin assignmnts are fiddled with



