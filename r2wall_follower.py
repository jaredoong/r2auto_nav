# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from signal import pthread_kill
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import time
import matplotlib.pyplot as plt

from custom_msgs.msg import Nfc, Button, Thermal, Flywheel, Launcher

# calibration parameters
slow_rotate = 0.7 # for rotating the bot slowly
fast_rotate = 0.9 # for rotating the bot quickly
speed_change = 0.20 # forward speed of the bot
stop_distance = 0.35 # stopping distance of the bot
threshold_temp = 35 # calibrated to temp of thermal object
total_nfc = 3 # number of detectable NFC in 1 round

# constants
occ_bins = [-1, 0, 100, 101]
scanfile = 'lidar.txt'
mapfile = 'map.txt'
FRONT = 0
FRONT_LEFT = 45
FRONT_FRONT_LEFT = 338
FRONT_RIGHT = 315
FRONT_FRONT_RIGHT = 22
LEFT = 90
RIGHT =270
BACK = 180

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)

        # create publisher for starting flywheels
        self.publisher_flywheel = self.create_publisher(Flywheel,'start_flywheel',10)

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning

        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.centered = False
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        # create subscription to check if nfc is detected
        self.nfc_subscription = self.create_subscription(
            Nfc,
            'nfc_found',
            self.nfc_callback,
            10)
        self.nfcfound = False
        self.nfc_subscription # prevent unused variable warning

        # create subscription to check if button has been pressed
        self.button_subscription = self.create_subscription(
            Button,
            'button_pressed',
            self.button_callback,
            10)
        self.buttonpressed = False
        self.button_subscription # prevent unused variable warning

        # create subscription to check if thermal object found
        self.thermal_subscription = self.create_subscription(
            Thermal,
            'thermal',
            self.thermal_callback,
            1)
        self.thermal_subscription # prevent unused variable warning
        self.thermalfound = False
        self.thermal_updated = False
        self.thermalimg = np.zeros((8,8))

        # create subscription to check if finished shooting
        self.launcher_subscription = self.create_subscription(
            Launcher,
            'finished_shooting',
            self.launcher_callback,
            10)
        self.launcher_subscription # prevent unused variable warning
        self.done_shooting = False

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.x_pos, self.y_pos, self.z_pos =  msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def nfc_callback(self, msg):
        # self.get_logger().info('In nfc_callback')
        self.nfcfound = msg.nfc_found
        self.get_logger().info('NFC detected: "%s"' % msg.nfc_found)

    def button_callback(self, msg):
        # self.get_logger().info('In button_callback')
        self.buttonpressed = msg.button_pressed
        self.get_logger().info('Button pressed: "%s"' % msg.button_pressed)

    def thermal_callback(self, msg):
        # self.get_logger().info('In thermal_callback')
        self.thermalimg = msg.thermal
        # data set into 8x8 array
        self.thermalimg = np.reshape(self.thermalimg,(8,8))
        # flip the data vertically
        self.thermalimg = np.flipud(self.thermalimg)
        print(self.thermalimg)
        self.thermal_updated = True

    def launcher_callback(self, msg):
        # self.get_logger().info('In launcher_callback')
        self.done_shooting = msg.finished_shooting
        self.get_logger().info('Finished shooting: "%s"' % msg.finished_shooting)

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        #time.sleep(1)
        self.publisher_.publish(twist)

    # algorithm to follow left wall
    def left_follow_wall(self,stop_d, speed, slow_r, fast_r):
        # create Twist object
        twist = Twist()

        # get distance from wall for each side
        # set out of range values to max distance of LIDAR
        front = np.nan_to_num(self.laser_range[FRONT], nan=3.5 ,posinf=3.5)
        frontright = np.nan_to_num(self.laser_range[FRONT_RIGHT], nan=3.5 ,posinf=3.5)
        frontleft = np.nan_to_num(self.laser_range[FRONT_LEFT], nan=3.5 ,posinf=3.5)
        frontfrontleft = np.nan_to_num(self.laser_range[FRONT_FRONT_LEFT], nan=3.5 ,posinf=3.5)
        left = np.nan_to_num(self.laser_range[LEFT], nan=3.5 ,posinf=3.5)
        right = np.nan_to_num(self.laser_range[RIGHT], nan=3.5 ,posinf=3.5)

        # for fine-tuning and debugging purposes
        # self.get_logger().info("Front: %.2f Frontleft: %.2f Frontright: %.2f" % (front, frontleft, frontright))

        # to calculate the diagonal stopping distance
        d = stop_d / math.cos(math.radians(45))

        # main logic for the wall follower algo, keeps track of left wall and follow it
        # wall detected if < d, else not detected

        # if no wall detected at all, turn left slightly to find wall
        if front > d and frontleft > d and frontright > d:
            if frontfrontleft > d:
                # self.get_logger().info("No obs at all, slow turning left to find the wall")
                twist.linear.x = speed*0.5
                twist.angular.z = fast_r
            # to prevent bot from crashing into tin can
            else:
                # self.get_logger().info("Obs at front front left, moving forward and away from wall")
                twist.linear.x = speed
                twist.angular.z = slow_r

        # wall detected in front only, turn right to find wall on left side
        elif front < d and frontleft > d and frontright > d:
            # self.get_logger().info("Wall in front only, fast turning right")
            twist.linear.x = 0.0
            twist.angular.z = -fast_r

        # wall detected on front left only, follow the wall
        elif front > d and frontleft < d and frontright > d:
            # check if bot is too close to the wall, if yes move away slightly
            # considered too close if left side is 25cm
            if frontleft < (0.25):
                # self.get_logger().info("Too close to left wall, fast turning right slightly")
                twist.linear.x = speed*0.5
                twist.angular.z = -fast_r
            # no changes needed, continue moving forward
            else:
                # self.get_logger().info("Correct distance, following wall")
                twist.linear.x = speed
                twist.angular.z = 0.0
        
        # wall detected on front right only, turn left to find wall
        elif front > d and frontleft > d and frontright < d:
            # self.get_logger().info("Wall at front right only, fast turning left to find wall")
            twist.linear.x = speed*0.5
            twist.angular.z = fast_r
        
        # wall detected on front left and front, turn right to avoid collision
        elif front < d and frontleft < d and frontright > d:
            # self.get_logger().info("Wall at front and front left, fast turning right to avoid collision")
            twist.linear.x = 0.0
            twist.angular.z = -fast_r

        # wall detected on front and front right, turn right to avoid collision
        elif front < d and frontleft > d and frontright < d:
            # self.get_logger().info("Wall at front and front right, fast turning right to avoid collision")
            twist.linear.x = 0.0
            twist.angular.z = -fast_r

        # wall detected on all 3 sides, turn right to avoid collision and keep wall on left side
        elif front < d and frontleft < d and frontright < d:
            # self.get_logger().info("Wall in all direction, fast right to avoid collision")
            twist.linear.x = 0.0
            twist.angular.z = -fast_r

        # wall detected on front left and front right, turn left to find wall
        elif front > d and frontleft < d and frontright < d:
            if left < d and right < d:
                # self.get_logger().info("Stuck in a corner, fast turn right to escape dead end")
                twist.linear.x = 0.0
                twist.angular.z = -fast_r
            else:
                # self.get_logger().info("Wall at front left and front right, slow turning left to find wall")
                twist.linear.x = speed*0.5
                twist.angular.z = fast_r

        # update velocity of turtlebot
        self.publisher_.publish(twist)

    def find_nfc(self):
        try:
            # keep track of number of NFC is has detected
            num_nfc_found = 0

            # ensure data being received from LIDAR before starting
            while (len(self.laser_range) == 0):
                self.get_logger().info("Fetching LIDAR data")
                rclpy.spin_once(self)

            # Start wall following algorithm once ready
            while rclpy.ok():
                self.left_follow_wall(stop_distance, speed_change, slow_rotate, fast_rotate)
                if self.nfcfound == True:
                    self.stopbot()
                    self.get_logger().info("NFC found")
                    num_nfc_found += 1
                    self.get_logger().info("Num of NFC found: %i" % num_nfc_found)
                    if num_nfc_found > total_nfc:
                        # move into loading phase only when bot has travelled one full round
                        break

                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

    def load_balls(self):
        self.get_logger().info("Loading balls phase started")
        try:
            while rclpy.ok():
                # allow the callback functions to run
                rclpy.spin_once(self)

                # bot remains stationary till balls are loaded
                if self.buttonpressed == False:
                    self.stopbot()
                    continue
                self.get_logger().info("Balls loaded, moving off in 2 seconds")
                # 2 sec delay added to allow TA to move out of the way
                time.sleep(2.0)
                self.get_logger().info("Moving off now")
                break

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

    def find_thermal(self):
        try:
            # Start and Format Figure 
            plt.rcParams.update({'font.size':16})
            fig_dims = (12,9) # figure size
            fig,ax = plt.subplots(figsize=fig_dims) # start figure
            pix_res = (8,8) # pixel resolution
            im1 = ax.imshow(self.thermalimg,vmin=15,vmax=35) # plot image, with temperature bounds
            cbar = fig.colorbar(im1,fraction=0.0475,pad=0.03) # colorbar
            cbar.set_label('Temperature [C]',labelpad=10) # temp. label
            fig.canvas.draw() # draw figure
            ax_bgnd = fig.canvas.copy_from_bbox(ax.bbox) # background for speeding up runs
            fig.show() # show figure

            while rclpy.ok():
                # allow the callback functions to run
                rclpy.spin_once(self)

                # prevent flywheel from starting before target is found
                flywheel = Flywheel()
                flywheel.start_flywheel = False
                self.publisher_flywheel.publish(flywheel)

                # waits for thermal data to be updated
                if self.thermal_updated == False:
                    # self.get_logger().info("Waiting for new data")
                    continue

                # Plotting in real time
                # self.get_logger().info('Redrawing image')
                fig.canvas.restore_region(ax_bgnd) # restore background (speeds up run)
                im1.set_data(np.reshape(self.thermalimg,pix_res)) # update plot with new temps
                ax.draw_artist(im1) # draw image again
                fig.canvas.blit(ax.bbox) # blitting - for speeding up run
                fig.canvas.flush_events() # for real-time plot
                #self.get_logger().info('Done redrawing image')

                # transpose image so that can check by columns
                thermal_data = np.transpose(self.thermalimg)
                # reset updated_variable once data has been used
                self.thermal_updated = False

                cols_found = []
                row_num = 0
                for row in thermal_data:
                    #self.get_logger().info("Checking col %i" % row_num)
                    col_num = 0
                    for temp in row:
                        #self.get_logger().info("Checking row %i, Temp is %.2f" % (col_num, temp))
                        if temp >= threshold_temp:
                            cols_found.append(row_num)
                            break
                        col_num += 1
                    row_num += 1
                # for checking with columns the object detected in    
                # print(("Columns found: {}").format(cols_found))

                if len(cols_found) != 0:
                    self.thermalfound = True
                else:
                    self.thermalfound = False

                if self.thermalfound == False:
                    self.get_logger().info("Thermal object not yet found")
                    # do wall following algo to find thermal object
                    self.left_follow_wall(stop_distance, speed_change, slow_rotate, fast_rotate)

                else:
                    # if thermal object is found, stop the bot
                    self.stopbot()
                
                # adjusting of position of bot relative to thermal object
                if len(cols_found) != 0:
                    # use the middle column to align
                    reference_col = cols_found[int(len(cols_found) / 2)]
                    if reference_col  < 3:
                        # turn left to centralise the object
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = 0.2*slow_rotate
                        time.sleep(1)
                        # self.get_logger().info("Turning left to centralise bot")
                        self.publisher_.publish(twist)
                        self.centered = False
                        time.sleep(0.5)
                    elif reference_col > 4:
                        # turn right to centralise the object
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = -0.2*slow_rotate
                        time.sleep(1)
                        # self.get_logger().info("Turning right to centralise bot")
                        self.publisher_.publish(twist)
                        time.sleep(0.5)
                        self.centered = False
                    else:
                        self.centered = True
                        self.stopbot()

                if self.centered:
                    self.get_logger().info("Centralised")
                    # if too far away, move closer to object
                    if (len(cols_found) < 3):
                        twist = Twist()
                        twist.linear.x = speed_change*0.5
                        twist.angular.z = 0.0
                        time.sleep(1)
                        self.publisher_.publish(twist)
                        time.sleep(1)
                    else:
                        self.stopbot()
                        self.get_logger().info("Correct distance away from object")
                        # move on to shooting phase once ready
                        break                    
                    
        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

    # function to start up the launcher and check if shooting is done
    def launcher(self):
        try:
            # Start up the flywheels
            flywheel = Flywheel()
            flywheel.start_flywheel = True
            self.publisher_flywheel.publish(flywheel)

            while rclpy.ok():
                rclpy.spin_once(self)
                # self.get_logger().info("Waiting for shooting to be finished")
                if self.done_shooting == True:
                    self.get_logger().info("Done shooting")
                    break

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.find_nfc()
    auto_nav.load_balls()
    auto_nav.find_thermal()
    auto_nav.launcher()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()