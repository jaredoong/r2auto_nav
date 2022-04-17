import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import busio
import board
import adafruit_amg88xx
import numpy as np
import pigpio

from .pn532 import *
from custom_msgs.msg import Nfc, Button, Thermal, Flywheel, Launcher

button_pin = 15
left_flywheel_pin = 21
right_flywheel_pin = 26
servo_pin = 23
num_balls = 3
total_nfc = 3 # number of detectable NFC in maze

GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.setup(left_flywheel_pin, GPIO.OUT, initial=0) # Set pin 37 to be an output pinwitches to high to start flywheels only when ready
GPIO.setup(right_flywheel_pin, GPIO.OUT, initial=0) # Set pin 40 to be an output pin, switches to high to start flywheels only when ready

# pigpio used to prevent servo jitter
p = pigpio.pi()
p.set_mode(servo_pin, pigpio.OUTPUT)
p.set_PWM_frequency(servo_pin, 50)

# Ranges from 500 (0) to 2500 (180)
# Setting the initial angle at 0
p.set_servo_pulsewidth(servo_pin, 1400)

class NFCFinder(Node):

    def __init__(self):
        super().__init__('nfc_finder')
        self.publisher_ = self.create_publisher(Nfc, 'nfc_found', 10)
        self.get_logger().info('Created publisher')
        self.get_logger().info('NFC function started')
        # self.nfc_callback()

    def nfc_callback(self):
        try:
            #pn532 = PN532_SPI(debug=False, reset=20, cs=4)
            pn532 = PN532_I2C(debug=False, reset=20, req=16)
            #pn532 = PN532_UART(debug=False, reset=20)

            #ic, ver, rev, support = pn532.get_firmware_version()
            #print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))

            # Configure PN532 to communicate with MiFare cards
            pn532.SAM_configuration()
            
            msg = Nfc()
            self.nfc_found = False
            num_nfc_found = 0

            #self.get_logger().info('Beginning while loop')
            while True:
                # Check if a card is available to read
                uid = pn532.read_passive_target(timeout=0.5)
                
                self.publisher_.publish(msg)
                self.get_logger().info("Num of NFC found: %i" % num_nfc_found)
                # Try again if no card is available.
                if uid is None:
                    self.get_logger().info('NFC not found')
                    continue
                self.get_logger().info('NFC found')
                msg.nfc_found = True
                num_nfc_found += 1
                self.publisher_.publish(msg)
                msg.nfc_found = False
                self.publisher_.publish(msg)
                time.sleep(2.0)

                if num_nfc_found > total_nfc:
                    break
                # let bot pass NFC to prevent double counting
                
                #self.get_logger().info('NFC found is %s' % msg.nfc_found)
                
        except Exception as e:
            print(e)
        #finally:
        #    GPIO.cleanup()

class ButtonPress(Node):

    def __init__(self):
        super().__init__('button')
        self.publisher_ = self.create_publisher(Button, 'button_pressed', 10)
        self.get_logger().info('Created publisher')
        self.get_logger().info('Button function started')
        # self.button_callback()

    def button_callback(self):
        try:
            
            msg = Button()
            self.button_pressed = False

            while True:
                if GPIO.input(button_pin) != GPIO.LOW:
                    self.get_logger().info("Button not pressed")
                    continue

                self.get_logger().info('Button pressed')
                msg.button_pressed = True
                self.publisher_.publish(msg)
                break
                
        except Exception as e:
            print(e)
        #finally:
        #    GPIO.cleanup()

class ThermalPhase(Node):

    def __init__(self):
        super().__init__('thermalphase')
        self.publisher_thermal = self.create_publisher(Thermal, 'thermal', 10)
        #self.get_logger().info('Created publisher')
        self.get_logger().info('Thermal function started')

        #create timer for publisher
        #time_duration = 0.1 # in seconds
        #self.timer = self.create_timer(time_duration, self.thermal_callback)

        self.publisher_launcher = self.create_publisher(Launcher, 'finished_shooting', 10)
        #self.get_logger().info('Created publisher')
        self.get_logger().info('Launcher function started')

        # create subscription to check if bot is centeralised is detected
        self.flywheel_subscription = self.create_subscription(
            Flywheel,
            'start_flywheel',
            self.flywheel_callback,
            10)
        self.aligned = False
        self.num_balls = num_balls # based on number of balls decided
        self.flywheel_subscription # prevent unused variable warning

        # Seting up the thermal sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(i2c)
        self.get_logger().info('Thermal Camera done setting up')

    # to receive info on whether can start launching process
    def flywheel_callback(self, msg):
        self.get_logger().info('In flywheel callback function')
        self.aligned = msg.start_flywheel
    
    # to send info on whether done launching
    def launcher_callback(self):
        self.get_logger().info('In launcher callback function')
        msg = Launcher()
        self.get_logger().info("Number of balls left = %i" % self.num_balls)
        if self.num_balls == 0:
            self.get_logger().info("No more balls left")
            msg.finished_shooting = True
        else:
            self.get_logger().info("Balls remaining")
            msg.finished_shooting = False
        self.publisher_launcher.publish(msg)

    def thermal_callback(self):
        self.get_logger().info('In thermal callback')
        try:
            msg = Thermal()
            data = self.amg.pixels

            count = 0
            for row in data:
                for col in row:
                    msg.thermal[count] = col
                    count += 1
            self.publisher_thermal.publish(msg)
            self.get_logger().info('Thermal camera msg published')
            
        except Exception as e:
            print(e)

    # function to allow bot to start and stop process of launching
    def start_thermal(self):
        self.get_logger().info("In start thermal function")
        try:
            #finds and align itself to the thermal object first
            #self.get_logger().info("In start thermal function, moving to callback")
            while self.aligned == False:
                self.thermal_callback()
                rclpy.spin_once(self)

            # start the flywheels up once ready
            #self.get_logger().info("Starting up the flywheels")
            GPIO.output(left_flywheel_pin, 1)
            GPIO.output(right_flywheel_pin, 1)
            # let the flywheel ramp up before starting servo
            time.sleep(0.2)

            # launch the ball with interval of 1 second
            while self.num_balls > 0:
                self.get_logger().info("Entering code to fire balls")
                # Setting the angle to 165
                self.get_logger().info("Firing ball")
                p.set_servo_pulsewidth(servo_pin, 800)
                time.sleep(0.5)
                # Setting the angle back to 15
                p.set_servo_pulsewidth(servo_pin, 1400)
                time.sleep(0.5)
                self.num_balls -= 1
                # updates whether done launching
                self.launcher_callback()
            
            # Off the flywheel
            GPIO.output(left_flywheel_pin, 0)
            GPIO.output(right_flywheel_pin, 0)
            self.get_logger().info("Done firing all balls")

        except Exception as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)

    # start with finding NFC
    nfc_pub = NFCFinder()
    nfc_pub.nfc_callback()
    nfc_pub.destroy_node()

    # once NFC found, wait for button press
    button_pub = ButtonPress()
    button_pub.button_callback()
    button_pub.destroy_node()

    # start the thermal function once button pressed
    thermal_pub = ThermalPhase()
    thermal_pub.start_thermal()
    thermal_pub.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()