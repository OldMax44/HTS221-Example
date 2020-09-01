
**************** UNDER CONSTRUCTION *********************************

This example is the HTS221 sensor connected to a Raspberry Pi Zero W.
With the availability of WiFi, I set up an Apache web server.  A C program
communicates with the HTS221 sensor via the I2C bus using the WiringPi
library.  It reads the claibration coefficients, takes sensor readings, and
runs the interpolation calculation.  It's an old technique, but the C program
then writes a simple XML file that is periodcally read by the served web
page.  I also included some radial guages to display the temperature and
humidity.

Note:  The future of the WiringPi library is undertermined now, but it was
included in my Raspbian distribution (Buster) and worked well.
