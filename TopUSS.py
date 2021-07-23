import RPi.GPIO as GPIO
import time

topTrigPin = 22
topEchoPin = 36
MAX_DISTANCE = 220          # define the maximum measuring distance, unit: cm
timeOut = MAX_DISTANCE*60   # calculate timeout according to the maximum measuring distance

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BOARD)      # use PHYSICAL GPIO Numbering
GPIO.setup(topTrigPin, GPIO.OUT)   # set topTrigPin to OUTPUT mode
GPIO.setup(topEchoPin, GPIO.IN)    # set topEchoPin to INPUT mode

def topPulseIn(pin,level,timeOut): # obtain pulse time of a pin under timeOut
    t0 = time.time()
    while(GPIO.input(pin) != level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    t0 = time.time()
    while(GPIO.input(pin) == level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    pulseTime = (time.time() - t0)*1000000
    return pulseTime
    
def topGetSonar():     # get the measurement results of ultrasonic module,with unit: cm
    GPIO.output(topTrigPin,GPIO.HIGH)      # make topTrigPin output 10us HIGH level 
    time.sleep(0.00001)     # 10us
    GPIO.output(topTrigPin,GPIO.LOW) # make topTrigPin output LOW level 
    pingTime = topPulseIn(topEchoPin,GPIO.HIGH,timeOut)   # read plus time of topEchoPin
    distance = pingTime * 340.0 / 2.0 / 10000.0     # calculate distance with sound speed 340m/s 
    return distance
