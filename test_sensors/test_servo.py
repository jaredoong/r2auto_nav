import time
import RPi.GPIO as GPIO

# Set pin numbering convention
GPIO.setmode(GPIO.BCM)

# Choose an appropriate pwm channel to be used to control the servo
servo_pin = 21

# Set the pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Initialise the servo to be controlled by pwm with 50 Hz frequency
p = GPIO.PWM(servo_pin, 50)

# Set servo to 90 degrees as it's starting position
p.start(7.5)

try:
   while True:
       angle = int(input("Enter 0 <= angle <= 180: "))
       p.ChangeDutyCycle((12.5-2.5) * (angle/180) + 2.5)
       
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()