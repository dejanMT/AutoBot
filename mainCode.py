#Imports
import RPi.GPIO as GPIO
import time
import requests

#Import code from an other py document
from ADCDevice import *
from topUSS import * #Top Ultrasonic

#PushBullet Mobile Notifications
API_KEY = 'yourApiKeyHere'
notificationFile = "notificationMessage.txt"

#The L298N (Motor Drive) GPIO Connections
Drive2Input1  = 29 #IN1
Drive2Input2  = 31 #IN2
Drive2Input3  = 33 #IN3
Drive2Input4  = 35 #IN3

#Infrared Sensor
leftSensor = 16
rightSensor = 18

#UltraSonic Sensor
#Front Pins
trigPin = 38
echoPin = 40

#TopPins
topTrigPin = 22
topEchoPin = 36

MAX_DISTANCE = 220  
timeOut = MAX_DISTANCE*60 

#Back LEDs
backLeds = 15

#ADC Device
ledPin = 11 #Top LEDs
adc = ADCDevice()

#Button
buttonPin = 32

proceed = False


#SetUp
def setUp():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    #Driver
    GPIO.setup(Drive2Input1, GPIO.OUT)
    GPIO.setup(Drive2Input2, GPIO.OUT)
    GPIO.setup(Drive2Input3, GPIO.OUT)
    GPIO.setup(Drive2Input4, GPIO.OUT)

    #Infrared Sensor
    GPIO.setup(leftSensor,GPIO.IN)
    GPIO.setup(rightSensor,GPIO.IN)

    #Front Ultrasonic Sensor
    GPIO.setup(trigPin, GPIO.OUT)
    GPIO.setup(echoPin, GPIO.IN)

    #Back Leds
    GPIO.setup(backLeds, GPIO.OUT)
    GPIO.setup(backLeds, GPIO.LOW)

    #ADC Device
    global adc
    if(adc.detectI2C(0x48)): # Detect the pcf8591.
        adc = PCF8591()
    elif(adc.detectI2C(0x4b)): # Detect the ads7830
        adc = ADS7830()
    else:
        print("No correct I2C address found, \n"
        "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
        "Program Exit. \n");
        exit(-1)
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(ledPin,GPIO.OUT)   # set ledPin to OUTPUT mode

    #Button
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) 

def pushMobileNotification(title, message):
        data={
            'type':'note',
            'title':title,
            'body':message
            }
        resp = requests.post('https://api.pushbullet.com/api/pushes', data=data, auth=(API_KEY,''))

#Movement
def forward():
    ''' Both motors will rotate parallel to the other making the vehicle move forward.'''
    GPIO.output(Drive2Input1, False)
    GPIO.output(Drive2Input2, True)
    GPIO.output(Drive2Input3, False)
    GPIO.output(Drive2Input4, True)

def right():
    ''' The left will stop rotating completely while the right motor keeps rotating forward making the vehicle move right. '''
    GPIO.output(Drive2Input1, False)
    GPIO.output(Drive2Input2, True)
    GPIO.output(Drive2Input3, True)
    GPIO.output(Drive2Input4, True)

def left():
    ''' The right will stop rotating completely while the left motor keeps rotating forward making the vehicle move left. '''
    GPIO.output(Drive2Input1, True)
    GPIO.output(Drive2Input2, True)
    GPIO.output(Drive2Input3, False)
    GPIO.output(Drive2Input4, True)

def stopAll():
    ''' All of the motors will stop rotating. Meanwhile, blink rear LEDs to simulate the hazard light in a vehicle. '''
    GPIO.output(Drive2Input1, False)
    GPIO.output(Drive2Input2, False)
    GPIO.output(Drive2Input3, False)
    GPIO.output(Drive2Input4, False)

    #When stopped blink rear LEDS
    GPIO.output(backLeds,GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(backLeds,GPIO.LOW)
    time.sleep(0.5)


def wireWalking():
    ''' In this function the vehicle will follow the line. The vehicle will distinguish between the line and the ground by the infrared sensous in attached in the front in the vehicle. '''
    # if both sensors are on then go forward
    if GPIO.input(leftSensor) == 1 and GPIO.input(rightSensor) == 1:
        forward()

    # if left sensor is on turn left
    elif GPIO.input(leftSensor) == 1 and GPIO.input(rightSensor) == 0: 
        left()  
             
    # if right sensor is on turn right
    elif GPIO.input(leftSensor) == 0 and GPIO.input(rightSensor) == 1: 
        right()

    #Else stop the motors and send a push notification to the application
    elif GPIO.input(leftSensor) == 0 and GPIO.input(rightSensor)  == 0:
        pushMobileNotification("Alert:", "Your Package has arrived!")
        stopAll() 
             
        
#ADC Device Function
def AdcDevice():
    value = adc.analogRead(0)    # read the ADC value of channel 0
        
    time.sleep(0.01)
    GPIO.output(ledPin,GPIO.LOW)
    if value >= 150:
        GPIO.output(ledPin,GPIO.HIGH)
    else:
        GPIO.output(ledPin,GPIO.LOW)

def destroyAdcDevice():
    adc.close()
    GPIO.cleanup()

#Front UltraSonic Sensor 
def pulseIn(pin,level,timeOut):
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
    
def getSonar():   
    GPIO.output(trigPin,GPIO.HIGH)      
    time.sleep(0.00001)     
    GPIO.output(trigPin,GPIO.LOW) 
    pingTime = pulseIn(echoPin,GPIO.HIGH,timeOut)
    distance = pingTime * 340.0 / 2.0 / 10000.0    
    return distance



def carrierLoaded():
    #When the user places an object update variable
    topDistance = topGetSonar()
    if topDistance < 15:
        proceed = True
    else:
        proceed = False

    return proceed

    
def mainCode():
    ''' This is the main function, where all of the above functions are combined in a single function. '''
    while True:
        AdcDevice()
        proceed = carrierLoaded()
        
        #If the object is placed on the vehicle start moving.
        if proceed == True:
            GPIO.output(backLeds,GPIO.HIGH)
            distance = getSonar()
            #When the obstacle is too close stop 
            if distance < 25:
                stopAll()
            #When the button is clicked
            elif  GPIO.input(buttonPin) == GPIO.LOW:
                print("----- Emergency Stop -----")
                pushMobileNotification("Warning", "Emergancy Stop Deployed.")
                stopAll()
                break
            #Else start following the line
            else:
                wireWalking()
        #Else stop all motors
        else:
            stopAll()

                    
    GPIO.cleanup()

    

if __name__ == '__main__':  
    print ('Program is starting...')
    setUp()
    try:
        mainCode()
    except KeyboardInterrupt:
        print ('Program stopped.')
        destroyAdcDevice()
        GPIO.cleanup()
    
    
