This is a OSD for FrSky Smart Port telemetry based on minimosd-extra.

To get this working you need:
- MinimOSD or MavlinkOSD
- UART to Smart Port Inverter http://git.razvi.ro/?p=minimosd-extra-frsky.git&a=blob&f=S.PORT_inveter.png
- FTDI 5v connector http://www.robofun.ro/conector-ftdi-5v

Installation:
- mkdir ArduCAM_OSD
- cd ArduCAM_OSD
- git clone git://git.razvi.ro/minimosd-extra-frsky.git .
- copy the the content of libraries in c:\Users\YourUser\Documents\Arduino\libraries\
- open ArduCAM_OSD.ino in Arduino
- select from Tools > Board > Arduino Pro or Pro Mini (5v, 16 MHz) w/ATmega328
- upload

Working:
- battery voltage from current sensor (FCS-40A)
- cell voltage (in temperature slot) from lipo voltage sensor (FLVSS)
- amperage from current sensor, old hub vfas (FAS-40/FAS-100/FAS-100-XT) and new smart port (FCS-40A)
- altitude form bario sensor (FVAS-02H)
- gps (GPS V2)
- low battery warning
- you can use Config. Tool to edit the settings

Todo :
- rssi ?

I used the following sources to write it:
- https://code.google.com/p/minimosd-extra
- https://github.com/zendes/SPort_OSD
- https://github.com/zendes/SPort
- https://code.google.com/p/telemetry-convert/wiki/FrSkySPortProtocol
- https://code.google.com/p/opentx/source/browse/trunk/src/telemetry/frsky_sport.cpp
