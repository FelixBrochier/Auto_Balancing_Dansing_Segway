import pyb
from pyb import Timer, ADC
from oled_938 import OLED_938
from mpu6050 import MPU6050

i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64, external_vcc=False, i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Challenge 5: K&F')
oled.display()

imu = MPU6050(1, False)

##MOTORS 
A1 = Pin('X3', Pin.OUT_PP)		
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				
B1 = Pin('X7', Pin.OUT_PP)		
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				


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


K_p = 20
K_i = 0.1
K_d = 0.25
set_point = 0 
alpha = 0.9 
tic = pyb.millis()
previous_pitch = imu.pitch()
pError_sum = 0 


## PITCH ANGLE
def pitch_estimate(pitch, dt,alpha):
    global pitch_dot
    theta = imu.pitch()
    pitch_dot= imu.get_gy()
    pitch = alpha*(pitch+pitch_dot*dt) + ( 1-alpha)*theta
    return (pitch,pitch_dot)


## PID CONTROLLER 
def PID(set_point, new_pitch, dt):
    global pError_sum 
    pError = set_point - new_pitch
    pError_dot = set_point - pitch_dot
    pError_sum = pError_sum + pError*dt

    pwm = K_p*pError + K_d*pError_dot + K_i * pError_sum 

    max(min(pwm, -100), 100)
    return pwm 


while True: 

    pyb.delay(100)
    toc = pyb.millis()
    dt = (toc-tic) * 0.001
    tic = pyb.millis()


    previous_pitch, previous_dot =  pitch_estimate(previous_pitch, dt, alpha)
    pwm = PID(0, previous_pitch, dt)

	
    if (pwm >= 0):	
        A_back(abs(pwm))
        B_back(abs(pwm))
    else:
        A_forward(abs(pwm))
        B_forward(abs(pwm))
