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

## NEW ALGO
# Wall follower around the entire maze first, then move to centre and travel up to down and left to right along centre line to map entire maze
# Dimension of wall kept in variable once first obj met
# If unable to locate NFC yet, travel in decrements of 50cm till object is found
# If unable to find thermal yet, navigate around wall of maze, turning everytime distance of opp wall suddenly increases it will rotate 180
# whenever it reaches the corners of the maze, will alos rotate back to direction it came from before turning back to start moving off. (thermal)

### TO FIX
# Currently unable to detect nfc while bypassing

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
import cmath
import time

from custom_msgs.msg import Nfc

# constants
rotatechange = 0.2
slowrotate = 0.70
fastrotate = 0.90
speedchange = 0.2
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.23
front_angle = 20 # min angle to prevent any collision
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
FRONT = 0
FRONT_LEFT = 45
FRONT_RIGHT = 315
LEFT = 90
RIGHT = 270
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
        # self.get_logger().info('Created publisher')

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
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

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.x_pos, self.y_pos, self.z_pos =  msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        # self.get_logger().info('X-axis: %.2f, Y-axis: %.2f, Z-axis: %.2f' % (self.x_pos, self.y_pos, self.z_pos))


    def occ_callback(self, msg):
        #self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        # log the info
        #self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        np.savetxt(mapfile, self.occdata)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def nfc_callback(self, msg):
        self.nfcfound = msg.nfc_found
        self.get_logger().info('NFC detected: "%s"' % msg.nfc_found)

    # for moving straight forward in current direction
    def move_forward(self):
        # start moving forward
        self.get_logger().info('Moving forward')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()

        # set the correct direction
        self.curr_dir = (self.curr_dir + rot_angle) % 360
        self.get_logger().info('Wanted dir is %i' % self.curr_dir)
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        # self.get_logger().info('Current: %f' % math.degrees(current_yaw))

        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # self.get_logger().info('Actual target yaw is %f' % target_yaw)
        
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        self.stopbot()
        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))

    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            ## lr2i = np.nanargmax(self.laser_range) Original Code!!!
            if np.take(self.laser_range, LEFT) > stop_distance:
                lr2i = LEFT
            elif np.take(self.laser_range, RIGHT) > stop_distance:
                lr2i = RIGHT
            elif np.take(self.laser_range, BACK) > stop_distance:
                lr2i = BACK
            else:
                lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.move_forward()

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    # algorithm to follow left wall
    def follow_wall(self):
        # create Twist object
        twist = Twist()

        # get distance from wall for each side
        front = np.nan_to_num(self.laser_range[FRONT], nan=3.5 ,posinf=3.5)
        frontright = np.nan_to_num(self.laser_range[FRONT_RIGHT], nan=3.5 ,posinf=3.5)
        frontleft = np.nan_to_num(self.laser_range[FRONT_LEFT], nan=3.5 ,posinf=3.5)

        self.get_logger().info("Front: %.2f Frontleft: %.2f Frontright: %.2f" % (front, frontleft, frontright))

        d = stop_distance / math.cos(math.radians(45))

        # main logic for the wall follower algo, keeps track of left wall and follow it
        # wall detected if < d, else not detected

        # if no wall detected at all, turn left slightly to find wall
        if front > d and frontleft > d and frontright > d:
            self.get_logger().info("No obs at all, slow turning left to find the wall")
            twist.linear.x = speedchange*0.5
            twist.angular.z = slowrotate

        # wall detected in front only, turn right to keep wall on left
        elif front < d and frontleft > d and frontright > d:
            self.get_logger().info("Wall in front only, fast turning right")
            twist.linear.x = 0.0
            twist.angular.z = -fastrotate

        # wall detected on front left only
        elif front > d and frontleft < d and frontright > d:
            # check if bot is too close to the wall, if yes move away slightly
            # considered too close if left side is d-3cm
            if frontleft < (d-0.05):
                self.get_logger().info("Too close to left wall, slow turning right slightly")
                twist.linear.x = speedchange*0.5
                twist.angular.z = -slowrotate
            # no changes needed, continue moving forward
            else:
                self.get_logger().info("Correct distance, following wall")
                twist.linear.x = speedchange
                twist.angular.z = 0.0
        
        # wall detected on front right only, turn left to find wall
        elif front > d and frontleft > d and frontright < d:
            self.get_logger().info("Wall at front right only, slow turning left to find wall")
            twist.linear.x = speedchange*0.5
            twist.angular.z = slowrotate
        
        # wall detected on front left and front, turn right to avoid collision
        elif front < d and frontleft <d and frontright > d:
            self.get_logger().info("Wall at front and front left, fast turning right to avoid collision")
            twist.linear.x = 0.0
            twist.angular.z = -fastrotate

        # wall detected on front and front right, turn left to avoid collision
        elif front < d and frontleft > d and frontright <d:
            self.get_logger().info("Wall at front and front right, fast turning left to avoid collision")
            twist.linear.x = 0.0
            twist.angular.z = fastrotate

        # wall detected on all 3 sides, turn right to avoid collision and keep wall on left side
        elif front < d and frontleft < d and frontright < d:
            self.get_logger().info("Wall in all direction, fast left to avoid collision")
            twist.linear.x = 0.0
            twist.angular.z = fastrotate

        # wall detected on front left and front right, turn left to find wall
        elif front > d and frontleft < d and frontright < d:
            self.get_logger().info("Wall at front left and front right, slow turning left to find wall")
            twist.linear.x = speedchange*0.5
            twist.angular.z = slowrotate

        # in event of unaccounted for cases, which should not happen
        else:
            self.get_logger().info("Unaccounted case, fix code")
        
        # update velocity of turtlebot
        self.publisher_.publish(twist)


    def mover(self):

        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            #  ensure data being received from LIDAR before starting
            while (len(self.laser_range) == 0):
                self.get_logger().info("Fetching LIDAR data")
                rclpy.spin_once(self)

            # Move forward once ready
            self.move_forward()
            
            while rclpy.ok():
                if self.nfcfound == True:
                    self.stopbot()
                    self.get_logger().info("NFC found")

                # add in if else part for thermal and button once ready
                else:
                    # self.get_logger().info("Entering wall following algo")
                    self.follow_wall()

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
                if self.loaded == True:
                    self.get_logger().info("Balls loaded, moving off in 5 seconds")
                    # 5 sec delay added to allow TA to move out of the way
                    time.sleep(5.0)
                    self.get_logger().info("Moving off now")
                    break

                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

    def find_objects(self):
        try:
            while rclpy.ok():
                if self.nfcfound == True and self.thermalfound == True:
                    self.get_logger().info("Both NFC and object found")
                    break
                elif self.nfcfound == True and self.thermal_found == False:
                    self.get_logger().info("NFC found, finding thermal now")
                    # self.find_thermal()
                    # add move forward temporarily so that bot can move
                    self.move_forward()
                else:
                    self.get_logger().info("Both not found, finding NFC now")
                    # self.find_nfc()
                    # add move forward temporarily so that bot can move
                    self.move_forward()
        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()
    #auto_nav.load_balls()
    #auto_nav.find_objects()
    #auto_nav.adjust_bot()
    #auto_nav.launcher()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
