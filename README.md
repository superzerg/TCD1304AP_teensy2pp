# TCD1304AP_teensy2pp

Control and read out directly the TCD1304AP CCD linear array with a teensy2pp.
It should work (with probably a bit of configuration) with an arduino Mega2560, but other arduino boards have too little RAM (the TCD1304AP has 3648 pixels, so to read all pixels at least 4k of memory is needed).

1) Installation / Usage

- Download and install the arduino software 1.6.9 (https://www.arduino.cc/en/Main/OldSoftwareReleases) 
- Donwload and install teensyduino  (https://www.pjrc.com/teensy/td_download.html)
- Download TCD1304AP_teensy2pp, open the TCD1304ap.ino file with the arduino software, compile and upload the sketch to your teensy2++.

2) Wirring
