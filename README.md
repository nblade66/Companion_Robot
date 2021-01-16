# Companion_Robot
To run the serial test, run driver.py.
You'll need to change the port to the one you're using in particular.
For Linux, you would go to the root folder and look into the /dev/ folder. There should be something like ttyUSBX, where X is the number. If the Arduino is the first thing you've plugged into the Nvidia, the port is probably /dev/ttyUSB0, which is what is already in the code. The last thing you'll need to do is set the "isWindowsOS" variable to False in the driver.py file.

Serial Communication:
The way it is set up right now is, the Nvidia is sending bytes over to the Arduino and then Arduino reads each byte and puts it into a queue (the cppQueue Library).

Dependencies:
Arduino:
You may need to install the cppQueue library for the Arduino

## Serial Codes
### Nvidia -> Arduino

	Modes:
	*	Distance	0000 0001
	*	Following	0000 0010
	*	Calibration 	0000 0011
	*	Waiting	    	0000 0000
	
	System:
	*	E-Halt Robot	0001 0000
	*	Clear Queue	0001 0001
	*	Set Precision	0001 01XX
	*	Pause Queue	
	*	Check State	0001 0010

	In Distance Mode: Use Encoder to measure distance; relies on encoder ticks; Extra bits indicate distance/angle.
	The units of the Value Bits for distance are in intervals of 5cm, so going forward has a max distance of about 64*5 = 320cm
	*	Forward         11XX XXXX
	*	Back            001X XXXX
	*	Left            01XX XXXX
	*	Right           10XX XXXX

	Following: Go in a direction until Nvidia says otherwise
	Extra bits indicate speed; i.e. the amount to subtract from the default PWM.
	If the value goes below 0 for Left and Right, it just results in the robot not moving.
	*	Forward-Right	110X XXXX (make ‘X XXXX’ = 0 for no turn)
	*	Forward-Left    111X XXXX
	*	Back/Halt	001X XXXX ('X XXXX' indicates speed; 0 = halt)
	*	Left	        01XX XXXX
	*	Right	        10XX XXXX
