# PoTFretFinder
Fork of the Paddle of Theseus code to run a special program to find physical fret positions via the PuTTY output

# Usage
This software works much like the PaddleFirmware, but in this case a FretFinder mode replaces the config menu mode completely.
Using the same gesture to enter config mode instead reboots the paddle into FretFinder mode, where it waits for a serial connection.

After connecting to serial, the fretFinder mode turns off the RotEnc RGB LED until the fretboard is held on the paddle.
If the fretboard is held where a fret should be placed, the LED lights up green, else it is off.

The serial connection prints the numerical value returned by the sensor, [ and soon the nearest fret, and the numerical value that should be used for the nearest fret].

If the user reboots the Paddle in FretFinder mode, it will reboot into standard play mode. Then again if the user uses the configMode gesture, the paddle will reboot back into FretFinder mode.