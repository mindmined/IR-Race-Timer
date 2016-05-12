# IR-Race-Timer
Track Race Times
This simple Race Timer is full compatible with the Hardware used in the RX8508-Pro by Marko Hoepken https://code.google.com/archive/p/rx5808-pro/

To simplify I only used Raceband channels 
On start up is is fixed with channel 7 Raceband. (to change goto Setup())
You can change the channel by using the up / down Buttons
Verfify that you have the right channel by checking video signal of the RX5808 
Press menu button for starting / stopping the timer
you could see the rssi bar changing while Time Tracking, if everthings right
change RSSI_STOP_TIMER 50 
if the signal strength is so high or low (max. 60)
Maybee you have to flash the RX8508-Pro Arduino code and do a RSSI calibration (haven't testet yet)

