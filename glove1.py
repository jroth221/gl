import RPi.GPIO as GPIO
import time
import PCF8591 as ADC
import math

# Set up GPIO pins for the ultrasonic sensor
GPIO_TRIGGER = 17
GPIO_ECHO = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# Set up GPIO pin for the vibration motor
MOTOR_PIN = 23 
GPIO.setup(MOTOR_PIN, GPIO.OUT)

# Set up GPIO pins for the button
BUTTON_PIN = 25
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set up GPIO pins for the buzzer
BUZZER_PIN = 21
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Set up GPIO pins for the temperature sensor
DO_PIN = 22
GPIO.setup(DO_PIN, GPIO.IN)

# Set up the ADC
ADC.setup(0x48)

# Set up global variables
sensors_on = True
button_pressed = False

def measure_distance():
    # Set trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # Wait for 10 microseconds
    time.sleep(0.00001)

    # Set trigger to LOW
    GPIO.output(GPIO_TRIGGER, False)

    start_time = time.time()
    stop_time = time.time()

    # Save start time
    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    # Save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    # Calculate time difference
    time_elapsed = stop_time - start_time

    # Calculate distance
    distance = (time_elapsed * 34300) / 2

    return distance

def turn_on_motor():
    GPIO.output(MOTOR_PIN, GPIO.HIGH)

def turn_off_motor():
    GPIO.output(MOTOR_PIN, GPIO.LOW)

def beep(x):
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    time.sleep(x)
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    time.sleep(x)

def check_temperature():
    analog_val = ADC.read(0)
    voltage = 5 * float(analog_val) / 255
    resistance = 10000 * voltage / (5 - voltage)
    temperature = 1/(((math.log(resistance / 10000)) / 3950) + (1 / (273.15+25)))
    temperature = temperature - 273.15
    print('Temperature = ', temperature, 'C')

    if temperature > 29:
        beep(2)

    return temperature

def button_callback(channel):
    global sensors_on, button_pressed
    button_pressed = True
    if sensors_on:
        sensors_on = False
        print('Sensors turned off')
    else:
        sensors_on = True
        print('Sensors turned on')

GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

while True:
    if button_pressed:
        button_pressed = False
        time.sleep(0.1)

    if sensors_on:
        distance = measure_distance()
        if distance < 25:
            turn_on_motor()
        else:
            turn_off_motor()

        check_temperature()

    time.sleep(0.1)
