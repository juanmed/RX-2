# RX-2 Force sensor

This repository contains code for reading data from the AIKOH RX-2 force sensor. See the example file rx20_read.py. 

The code for reading rx-2 force sensor data can be run in any computer, but since this was intended to be used in a Raspberry Pi 3 B+, some code (like PWM outputs) is specific to this board. 

If you are not interested in the code pertaining the RPi 3B+, ignore it.

Requirements
* pyserial
* RPi.GPIO
* matplotlib
* numpy

This can be installed by doing from your terminal

```bash
pip install -r requirements.txt
```

