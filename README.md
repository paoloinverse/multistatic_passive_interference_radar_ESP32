# Multistatic passive interference radar fully based on the ESP32 DevKit onboard wifi radio.

Do you really need to connect your ESP32 SoC to external movement detector sensors such as infrared PIR, ultrasonic or dedicated radar sensors, 
when the ESP32 itself can work as a passive radar *without any additional hardware* ?

Use the ESP32 wifi 802.11 radio as a Multistatic radar based on multipath interference. Will detect human intruders when they cross the path 
between the ESP32 and other access points or freely move inside rooms / buildings at a reasonable distance from the ESP32.

This code fully classifies all of the other access points in range and uses them as the transmitters in a multistatic passive radar system. 
When cleverly used, this system can detect both the movement of an intruder *and* its approxymate position in the transmitters constellation.
The system is passive and scans the available channel, using the beacon signals as broadcast by each access point in range. 
When properly tuned, this system is stable even when using weak and far access points. 

The number of transmitters used is fully configurable, by default it's set to 4 but can be increased runtime up to 8, or up to larger numbers by 
tweaking the library header file. 

Refer to the included .ino example, the basic usage is pretty simple. Feel free to experiment, this code works pretty well inside buildings and I expect it work 
even in the open field, as long as a line of sight path is corred at least once. 

Indeed you will notice the strongest variations are when an intruder crosses the line-of-sight path between the ESP32 and the other access points
used as transmitters. However, appreciable variations will be detected when indirect / reflected paths are crossed!

The example code outputs the signals over the serial port, Arduino IDE can be used to produce a plot in order to test the capabilities of the radar code and to 
determine the optimum parameters. 
Most of the code is fully automated and preconfigured, although the core parameters can be modified by editing the .h file or configured runtime via library calls.
Functions are provided to tweak the most impost important parameters, including the RSSI threshold, alarm threshold, the debugging level, plot print, and how many transmitters to use. 

The example contains a serial commands function with a very simple way to manually change runtime parameters by sending commands via the serial port from the arduino IDE


Commented example plot follows:
![multistatic_interference_radar_sample_4](https://user-images.githubusercontent.com/62485162/147374363-2aff0c62-4fda-491a-add7-f48e8588a33b.png)

Access point names censored, the uncensored parts contain both RSSI and filtered signal variance data for each transmitter.

The plot is in arbitrary units derived from the received signal variance data, it is contructed by a relatively complex digital filter entirely built in integer math, with a low computational expense.

Please note the code is mostly self-configuring and can autonomously take care of the common problems and failures typically encountered in a wifi based infrastructure, incuding faults affecting the nearby access points and stations. Yet, I warmly recommend to take a few minutes to tweak the parameters (minimum acceptable RSSI and used transmitters number) for your specific environment. I also recommend you set the minimum acceptable RSSI much lower than the average RSSI of the weakest signal you're receiving, because when a signal is lost or deemed unceeptably low, it will be replaced... and variance data will have to be reconstructed. This will take some time.

That being said, feel free to mess with the library internal parameters (such as buffer and filter sizes): if you find anything interesting and worth of notice, I'd be pleased to discuss it with you. 


At the moment, the library performs one full scan per iteration, which is somewhat slow. If you need faster response times, use this other library instead: 
https://github.com/paoloinverse/bistatic_interference_radar_esp
