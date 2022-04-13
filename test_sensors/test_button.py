import time
import RPi.GPIO as GPIO

# Any unused GPIO pin can be used as the button pin
button_pin = 15

# Set pin numbering convention
GPIO.setmode(GPIO.BCM)

# Set button pin to be an input pin, with pull up resistor enabled
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) 

try:
    while True:
        if GPIO.input(button_pin) != GPIO.LOW:
            continue
        print('Button pressed')
        break
        
except Exception as e:
    print(e)
finally:
    GPIO.cleanup()