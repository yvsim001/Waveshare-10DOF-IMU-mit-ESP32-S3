3D cube orientation contains MATLAB scripts to receive sensor data from an IMU sensor 
via UART, to determine the 3D orientation based on that data, and to apply the 
orientation to a 3D cube (so that the cube's orientation matches that of the IMU sensor).
For the UART communication between PC and the Sensor, the serial terminal RealTerm 
(https://sourceforge.net/projects/realterm/) is used, and thus has to be installed on 
the PC running the 3D cube program. During installation, make sure to uncheck the box 
regarding installation of the AUX wrapper. When asked for registration, click on “yes”.

This packet contains the following files:

- OpenCOMport: 
	opens the com port based on the passed com port number and the baudrate

- CloseCOMport: 
	closes the com port

- GetSerialData: 
	used to read UART data received via RealTerm. The textscan commands in 
	lines 36 and 38 have to be adapted according to the structure of the received UART data.
	It is recommended to modify the code for sending out the sensor data (on the 
	microcontroller) so that the sensor data does not contain any text, but only the data 
	elements separated by a single empty space. Each set of data elements shall be 
	completed by a newline character (\n)
 
- Orientation3Dcube: 
	contains the 3D cube and the basic structure for the 3D cube 
	animation. The pieces of code that need to be added, are marked by TODO comments
