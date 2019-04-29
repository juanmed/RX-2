#  Author: Juan Medrano
#  Version: 0.0
#  Date: 2019 Apr 29
#  Description: Simulate a PWM signal from an RC transmitter, 50Hz and duty cycle 
#  between 5% (1ms) and 10% (2ms). This signal is feed to an ESC. The ESC drives 
#  a rotor which is attached to the force sensor. The force values are then measured
#  by varying the PWM duty cycle.
#  Repository: github.com/juanmed/rx-20



import serial
import numpy as np
import RPi.GPIO as GPIO
import matplotlib
import re
import time
import csv

import matplotlib.pyplot as plt

def init_serial(port, baudrate):
    # configure the serial connections (the parameters differs on the device you are connecting to)
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )   

    if(ser.isOpen()):

        # Read program version
        ser.write('RDVR\r')
        time.sleep(0.1)

        version = ''
        while ser.inWaiting() > 0:
            version += ser.read(1)

        # Read allowable overload
        ser.write('RDMDL\r')
        time.sleep(0.1)

        overload = ''
        while ser.inWaiting() > 0:
            overload += ser.read(1)

        # Read current value
        ser.write('RDF0\r')
        time.sleep(0.1)

        val = ''
        while ser.inWaiting() > 0:
            val += ser.read(1)


        print('======================================================\n'+
			  '       AIKOH RX-2 Force Sensor Interface program \n'+
              'Serial Port: {} \n'.format(ser.name)+
              'Baudrate: {}\n'.format(baudrate)+
              'Program version: {}\n'.format(version)+
              'Allowable Overload {}\n'.format(overload)+
              'Current read {}\n'.format(val)+
			  '======================================================\n')
    else:
        print('Problem while opening {}'.format(port))
        ser = None

    return ser

def init_rpi():
	# read board info
	print("==========================")
	print("      RPi BOARD INFO ")
	rpi_info = GPIO.RPI_INFO
	for key in rpi_info:
		print("{} : {}".format(key, rpi_info[key]))
	

	# use board numbering
	GPIO.setmode(GPIO.BOARD)
	mode = GPIO.getmode()
	print("RPi.GPIO mode set to: {}".format(mode))
	GPIO.setwarnings(False)
	print("Warnings disabled!\n")
	print("==========================\n")
	
def get_data(dt_range, n, pwm_channel):
	"""
	@dt_range range of duty cycles to generate 
	@n  number of force readings to make per duty cycle value
	@pwm_channel RPi.GPIO PWM object over which the PWM signal will be 
				 generated
	@return list of tuples. Each element in the tuple is (mean, std, duty cycle)
	"""
	point_list = list()
	pwm_channel.start(0)
	for dt in dt_range:
		
		# sweep all duty cycles
		print('Sample with duty cycle =  {}%'.format(dt))
		pwm_channel.ChangeDutyCycle(dt)
		time.sleep(1)			# wait some time for rotor speed to settle
		
		data = list()
		request = True                  # flag to request data
		
		while len(data) < n:

			# request data
			if request:
				gauge.write('RDF0\r')
				request = False
				time.sleep(0.1)

			# check if data is available
			if(gauge.inWaiting()):
				val = ''
				while gauge.inWaiting() > 0 : 
				    val += gauge.read(1)
				data.append(val)
				request = True

		data = [float(re.findall(r"[-+]?\d*\.\d+|\d+", value)[0]) for value in data]
		data = np.array(data)		
		mean = np.mean(data)
		std = np.std(data, ddof = 1)

		point = [mean,std, dt]
		point_list.append(point)
	
	return point_list
		
		
		
	

# init gauge
gauge = init_serial('/dev/ttyUSB0', 38400)

# init rpi 
init_rpi()

# pwm params
freq = 50					# PWM frequency
dt_min = 5					# minimum duty cycle
dt_max = 10					# maximum duty cycle
dt_step = 0.1

# Configure pwm
GPIO.setup(35, GPIO.OUT) 	# configure pin 12 as output for pwm
p1 = GPIO.PWM(35,freq)		# Pin 35, 

n = 50						# read 100 force values per duty cycle value

a = raw_input('>> Press "S" to start sampling, press "C" to Cancel: ')
if (a == "S"):

	print(">> Sampling will start...")
	print(" *******  WARNING ********\n" + 
	      " If a rotor will spin,\n"+ 
	      " make sure you are at a \n" + 
 	      " safe distance! \n"+
	      " *******  WARNING ********\n")
	raw_input("Test will start 5 seconds after you press any key...")
	time.sleep(5)
	print('>> Test start...')
	dt_range = np.arange(dt_min, dt_max, dt_step)		# make duty cycle sweep range
	data = get_data(dt_range, n, p1)
	print(">> Sampling Finished.")

	# recover mean, std
	force_vals = [point[0] for point in data]
	force_std = [point[1] for point in data]
	dt = [point[2] for point in data]

	# plot
	fig0 = plt.figure(figsize=(20,10))
	ax0 = fig0.add_subplot(111)
	ax0.plot(dt, force_vals, color = 'r', linestyle = '-', label = 'Force (mean) {N}')
	ax0.plot(dt, force_std, color = 'g', linestyle = '-', label = 'Force (std) {N}')
	ax0.legend(loc='upper right', shadow=True, fontsize='medium')
	ax0.set_title('Force Reading using AIKOH RX-2')
	ax0.set_ylabel('Force {N}')
	ax0.set_xlabel('n')


	
elif(a == "C"):
	print(">> Sampling canceled. Bye!")
else:
	print(">> Option not recognized. Bye!")


p1.stop()					# stop pwm
print(">> PWM Signal stoped.")
GPIO.cleanup()
print(">> RPi resources freed.")
gauge.close()
print(">> Serial port closed.")


# Plot after all signals are off
plt.show()


filename = raw_input(">> Enter file name to save data: ")

# save data in csv file
with open(filename, 'a') as csvfile:
	writer = csv.writer(csvfile)
	writer.writerows(data)

csvfile.close()
print(">> File {} saved.Bye!".format(filename))


	
		







