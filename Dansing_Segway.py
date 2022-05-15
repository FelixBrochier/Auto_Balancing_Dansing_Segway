import pyb
from pyb import Pin, Timer, ADC, DAC, LED
from array import array			# need this for memory allocation to buffers
from oled_938 import OLED_938	# Use OLED display driver
from audio import MICROPHONE
from neopixel import NeoPixel
import random

#  The following two lines are needed by micropython
#   ... must include if you use interrupt in your program
import micropython
micropython.alloc_emergency_exception_buf(100)

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64,
    external_vcc=False,
    i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Dancing with stabilisers')
oled.display()

# create neopixel object
np = NeoPixel(Pin("Y12", Pin.OUT), 8)

A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

# Configure timer 2 to produce 1KHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

def A_forward(value):
	A1.low()
	A2.high()
	motorA.pulse_width_percent(value)

def A_back(value):
	A2.low()
	A1.high()
	motorA.pulse_width_percent(value)
	
def A_stop():
	A1.high()
	A2.high()
	
def B_forward(value):
	B2.low()
	B1.high()
	motorB.pulse_width_percent(value)

def B_back(value):
	B1.low()
	B2.high()
	motorB.pulse_width_percent(value)
	
def B_stop():
	B1.high()
	B2.high()


# define ports for microphone, LEDs and trigger out (X5)

b_LED = LED(4)		# flash for beats on blue LED

def flash():		# routine to flash blue LED when beat detected
    i = random.randrange(0,64)
    j = random.randrange(0,64)
    k = random.randrange(0,64)
    s = random.randrange(0,5)

    for i in range(random.randrange(1,8)):
		np[i] = (1*i, 1*j, 1*k)
		np.write()
        # pyb.delay(10)

    if s == 0: # Sequence is FB
        A_forward(400)
        B_forward(400) # move forward

        pyb.delay(300)

        A_forward(0)
        B_forward(0)
    
        pyb.delay(300)

        A_back(400)
        B_back(400) # move forward

        pyb.delay(300)

        A_back(0)
        B_back(0)

        pyb.delay(300)

    elif s ==1:  # Sequence LB
        A_forward(400)
        B_back(400) # move left

        pyb.delay(500)

        A_forward(0)
        B_back(0)
    
        pyb.delay(300)


    elif s == 2: # Sequence is RF
        B_forward(400)
        A_back(400) # Move right

        pyb.delay(500)

        A_forward(0)
        B_back(0)
    
        pyb.delay(300) 

    elif s == 3: #  Sequence U-turn

        B_forward(800)
        A_back(800) # move forward

        pyb.delay(1000)

        A_forward(0)
        B_back(0)

        pyb.delay(300)


    elif s == 4: # Sequence BR
        B_forward(400)
        A_forward(400) # move forward

        pyb.delay(300) 


        B_forward(400)
        A_back(400) # Move right

        pyb.delay(500)

        A_forward(0)
        B_forward(0)
    
        pyb.delay(300)


    elif s == 5: # Sequence LF

        A_forward(400)
        B_back(400) # move left

        pyb.delay(500)

        A_forward(400)
        B_forward(400)

        pyb.delay(300)

        A_forward(0)
        B_forward(0)
    
        pyb.delay(300)






# Create timer interrupt - one every 1/8000 sec or 125 usec
pyb.disable_irq()
sample_timer = pyb.Timer(7, freq=8000)	# set timer 7 for 8kHz

N = 160				# number of sample to calculate instant energy
mic = ADC(Pin('Y11'))
audio = MICROPHONE(sample_timer, mic, N)
pyb.enable_irq(True)

# Calculate energy over 50 epochs, each 20ms (i.e. 1 sec)
M = 50						# number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 10.0		# threshold for c to indicate a beat
MIN_BEAT_PERIOD = 500	# no beat less than this

# initialise variables for main program loop
e_ptr = 0					# pointer to energy buffer
e_buf = array('L', 0 for i in range(M))	# reserve storage for energy buffer
sum_energy = 0				# total energy in last 50 epochs
oled.draw_text(0,20, 'Ready to GO')	# Useful to show what's happening?
oled.display()
pyb.delay(100)
tic = pyb.millis()			# mark time now in msec

while True:				# Main program loop
	if audio.buffer_is_filled():		# semaphore signal from ISR - set if buffer is full
		
		# Fetch instantaneous energy
		E = audio.inst_energy()			# fetch instantenous energy
		audio.reset_buffer()			# get ready for next epoch

		# compute moving sum of last 50 energy epochs with circular buffer
		sum_energy = sum_energy - e_buf[e_ptr] + E
		e_buf[e_ptr] = E			# over-write earliest energy with most recent
		e_ptr = (e_ptr + 1) % M		# increment e_ptr with wraparound - 0 to M-1
		average_energy = sum_energy/M

		# Compute ratio of instantaneous energy/average energy
		c = E/average_energy




		if (pyb.millis()-tic > MIN_BEAT_PERIOD):	# if longer than minimum period
			if (c>BEAT_THRESHOLD):		# look for a beat
				flash()					# beat found, flash blue LED
				tic = pyb.millis()		# reset tic
		buffer_full = False				# reset status flag
