# Sample program configuring and using pwm gpio pins and pwm 
# in raspberri Pi 3 Model B+

import RPi.GPIO as GPIO

# read board info
print("==========================")
print("      RPi BOARD INFO ")
rpi_info = GPIO.RPI_INFO
for key in rpi_info:
	print("{} : {}".format(key, rpi_info[key]))
print("==========================\n")

# use board numbering
GPIO.setmode(GPIO.BOARD)
mode = GPIO.getmode()
print("RPi.GPIO mode set to: {}".format(mode))
GPIO.setwarnings(False)
print("Warnings disabled!\n")


# Configure io pins
GPIO.setup(35, GPIO.OUT) 		# configure pin 12 as output for pwm

# configure and start pwm
p1 = GPIO.PWM(35, 1000)			# pin 12, freq = 0.5hz
p1.start(30)					# start with duty cycle = 30$
a =raw_input('Press any key to stop\n')
p1.stop()

# Clean up and free resources
GPIO.cleanup()
print("Stopped and resources cleaned up, bye!")
