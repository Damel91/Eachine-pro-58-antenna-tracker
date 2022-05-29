# Eachine-pro-58-antenna-tracker
An RSSI antenna tracker based on the eachine pro58


Tracker brain is the source for the arduino nano

wifi_firmware is the source for the esp8266

Into the PCB folder you can find the PCB wich is ready to use with the press and peel tecnique,
also you can find the pinout of the eachine pro 58

The 3D folder is a mess, but there are all the file included go to thingiverse to find the freecad file to modify the project and see the images of the project

This antenna tracker is based on PID controller and single antenna tracking is experimental so use the manual settings

To configurate the tracker use the GUI app that you need to install into your android smartphone

The project is designed also to work with a bmp280 module connected to the i2c bus
The analog pin of the ESP8266 is used to read the battery voltage, so use a trimmer to calibrate the voltage

some esp8266 analog pin has 1V max others 3,3V, use the jumper to disconnect the trimmer output from the analog pin and calibrate the maximum voltege with the maximum level for the esp8266

The step down module can work from 2S LIPO to 6S LIPO and the code will automatically recognize the battery status and it's cell number.
