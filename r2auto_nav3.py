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
rotatechange = 0.1 # reduced speed while testing
speedchange = 0.1 # reduced speed while testing
obstaclespeed = 0.2
uturnforwardspeed = 0.1
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.3
turtlebot_length = 0.3
full_width_length = 5.0 - 2*stop_distance - turtlebot_length
angle_error = (2.0/180) * math.pi
front_angle = 20 # min angle to prevent any collision
side_angle = 10
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
FRONT = 0
LEFT = 90
RIGHT = 270
BACK = 180

# global variables
turn_direction = LEFT 


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

        #self.nfc_subscription = self.create_subscription(
        #    Nfc,
        #    'nfc_found',
        #    self.nfc_callback,
        #    10)
        #self.nfc_subscription # to prevent unused variable warning

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
        self.x_pos = 0
        self.starting_x_pos = 0
        self.starting_y_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        self.curr_dir = 0
        self.not_set = True
        self.x_travelled_dist = 0.0 
        self.y_travelled_dist = 0.0
        self.size = 0.0
        self.finished_mapping = False
        # move thermal_found to thermal sub once it is created
        self.thermal_found = False
        # move button_pressed to button sub once it is created
        self.button_pressed = False
        
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
        
    # to set the initial position so that the distance travelled can be tracked
    def get_initial_pos(self):
        self.starting_x_pos = self.x_pos
        self.starting_y_pos = self.y_pos
        self.not_set = False
        self.get_logger().info('Starting x position = %.2f' % self.starting_x_pos)
        self.get_logger().info('Starting y position = %.2f' % self.starting_y_pos)

    # for measuring the distance travelled along both x and y axis
    def travelled(self):
        self.x_travelled_dist = abs(self.x_pos - self.starting_x_pos)
        self.y_travelled_dist = abs(self.y_pos - self.starting_y_pos)
        # self.get_logger().info('Total x distance travelled = %.2f' % self.x_travelled_dist)
        # self.get_logger().info('Total y distance travelled = %.2f' % self.y_travelled_dist)

    # check if the robot is at the end of the maze
    def edge_reached(self):
        if self.x_travelled_dist < full_width_length and self.y_travelled_dist < full_width_length:
            return False
        # reset the initial position
        self.starting_x_pos = self.x_pos
        self.starting_y_pos = self.y_pos
        # reset value of distance travelled
        self.x_travelled_dist = 0.0
        self.y_travelled_dist = 0.0
        return True

    ### NO LONGER NEEDED
    # to check if the obstacle can be bypassed in the current direction
    #def cannot_bypass(self):
    #    if (abs(self.y_pos - self.starting_y_pos) > 4.0):
    #        self.get_logger().info("Current Y travelled is %f" % abs(self.y_pos - self.starting_y_pos))
    #        self.get_logger().info('Unable to bypass, trying the other direction')
    #        return True
    #    return False

    # for moving straight forward in current direction
    def move_forward(self):
        # start moving forward
        self.get_logger().info('Moving forward')
        twist = Twist()
        twist.linear.x = obstaclespeed
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

    # to travel a fixed distance in a specific direction
    def travel_distance(self, direction, distance):
        if direction == LEFT or direction == RIGHT:
            curr_pos = self.y_pos
            while (abs(curr_pos - self.y_pos) < distance):
                rclpy.spin_once(self)
                if np.average(self.laser_range[0]) <= stop_distance:
                    self.stopbot()
                    self.get_logger().info('Obs in front, unable to allocate more travel distance')
                    break
        else:
            curr_pos = self.x_pos
            while (abs(curr_pos - self.x_pos) < distance):
                rclpy.spin_once(self)
                if np.average(self.laser_range[0]) <= stop_distance:
                    self.stopbot()
                    self.get_logger().info('Obs in front, unable to allocate more travel distance')
                    break
        self.get_logger().info('Extra travel distance met')

    # function to bypass the obstacle to continue moving
    def bypass_obstacle(self):
        # to keep track of direction to turn back to once done
        initial_dir = self.curr_dir
        # variable to prevent turtlebot from exiting function even before it starts moving
        moved_off = False
        extra_distance = 0.20
        # keep track of turns made to bypass
        bypass_turns = []

        # check the side with larger clearance and turn that way to bypass
        if (np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT)):
            bypass_direction = LEFT
            bypass_opp_dir = RIGHT
        else:
            bypass_direction = RIGHT
            bypass_opp_dir = LEFT
        #if wall_location == LEFT:
        #    bypass_direction = RIGHT
        #    bypass_opp_dir = LEFT
        #else:
        #    bypass_direction = LEFT
        #    bypass_opp_dir = RIGHT

        # checks which axis to be used to keep track of whether done
        if self.curr_dir == LEFT or self.curr_dir == RIGHT:
            initial_pos = self.x_pos
            x_axis = True
        else:
            initial_pos = self.y_pos
            x_axis = False
        self.get_logger().info('Initial position is: %.2f' % initial_pos)
        # turn the turtlebot before starting adopted Pledge algorithm
        self.get_logger().info('Turtlebot turned %s to start Pledge Algo' % bypass_direction)
        self.rotatebot(bypass_direction)
        bypass_turns.append(bypass_direction)
        # setting the first avg wall distance
        prev_wall_avg_side_distance = np.mean(np.ma.masked_invalid(self.laser_range[bypass_opp_dir-side_angle:bypass_opp_dir+side_angle]))
        self.get_logger().info("First wall distance is %f" % prev_wall_avg_side_distance)
        
        # check if bot might be at edge of wall
        self.get_logger().info("Checking if almost over obs")
        if (bypass_opp_dir == LEFT):
            back_angle_dis = np.mean(np.ma.masked_invalid(self.laser_range[bypass_opp_dir:bypass_opp_dir+front_angle]))
            front_angle_dis = np.mean(np.ma.masked_invalid(self.laser_range[bypass_opp_dir-front_angle:bypass_opp_dir]))
        else:
            front_angle_dis = np.mean(np.ma.masked_invalid(self.laser_range[bypass_opp_dir:bypass_opp_dir+front_angle]))
            back_angle_dis = np.mean(np.ma.masked_invalid(self.laser_range[bypass_opp_dir-front_angle:bypass_opp_dir]))
        self.get_logger().info("Back angle distance is %f" % back_angle_dis)
        self.get_logger().info("Front angle distance is %f" % front_angle_dis)
        # check if avg distance of back angle is smaller than avg distance of front angle, if yes, bot almost past obs
        # 1.5* added to prevent insignificant differences from triggering code
        if (back_angle_dis*1.5 < front_angle_dis):
            self.get_logger().info("Almost over obs")
            self.move_forward()
            self.travel_distance(self.curr_dir, extra_distance)
            self.stopbot()
            self.rotatebot(bypass_opp_dir)
            bypass_turns.append(bypass_opp_dir)
            # reset the distance of the side wall
            rclpy.spin_once(self)
            prev_wall_avg_side_distance = np.mean(np.ma.masked_invalid(self.laser_range[bypass_opp_dir-side_angle:bypass_opp_dir+side_angle]))
            self.get_logger().info("New first wall distance is %f" % prev_wall_avg_side_distance)
            self.get_logger().info("In pos for main loop")
        # start moving forward after turn is made
        self.move_forward()
        
        # Loop to continuously check till turtlebot is done
        # Breaks out of loop automatically when done
        while (True):

            # to update the laser_range values
            rclpy.spin_once(self)
            if x_axis == True:
                curr_pos = self.x_pos
            else:
                curr_pos = self.y_pos
            # check if the turtlebot has reached the other side after moving off, with an allowance of 3 cm
            if (moved_off and (abs(initial_pos - curr_pos) <= 0.03)):
                # turtlebot has successfully navigated around the obstacle, exiting function to resume normal navigation
                self.get_logger().info('Turtlebot successfully navigated around the obstacle')
                self.stopbot()
                # making sure turtlebot turns back to the initial direction before moving again
                while self.curr_dir != initial_dir:
                    self.get_logger().info('Turning to initial direction')
                    self.rotatebot(bypass_direction)
                    rclpy.spin_once(self)
                return None

            # check if there is obstacle in front blocking the way
            if self.laser_range.size != 0:
                # check distances in front of TurtleBot and find values less
                # than stop_distance
                front_wall = (self.laser_range[front_angles]<float(stop_distance)).nonzero()

                if(len(front_wall[0])>0):
                    # stop moving
                    self.stopbot()
                    self.get_logger().info('Obstacle encountered infront')
                    # to update the variable self.y_travelled_dist and self.x_travelled_dist
                    self.travelled()

                    ## NO LONGER NEEDED
                    #self.get_logger().info("Y dist travelled is %f" % abs(self.y_pos - self.starting_y_pos))
                    #if self.cannot_bypass():
                    #    # swap the direction of bypass and turn to the back
                    #    bypass_direction, bypass_opp_dir = bypass_opp_dir, bypass_direction
                    #    self.get_logger().info('Direction swapped')
                    #    self.rotatebot(BACK)
                    #    # clear the bypassing turn tracker
                    #    bypass_turns = []
                    #    self.move_forward()
                    #    continue

                    # exit bypass function if edge of maze is reached
                    if x_axis:
                        travelled_dist = self.y_travelled_dist
                    else:
                        travelled_dist = self.x_travelled_dist

                    if (travelled_dist >= full_width_length):
                        self.get_logger().info('Reached the edge of the maze while bypassing obs')
                        break
                    if np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT):
                        self.rotatebot(LEFT)
                        bypass_turns.append(LEFT)
                    else:
                        self.rotatebot(RIGHT)
                        bypass_turns.append(RIGHT)

                    self.move_forward()
            
            # main portion on sticking to the wall
            # calculate the current avg distance from wall from front and side
            # self.get_logger().info('Updating current average wall distance')
            curr_wall_side_distances = self.laser_range[bypass_opp_dir-side_angle:bypass_opp_dir+side_angle]
            # change all inf to max range of LIDAR
            curr_wall_side_distances[np.isposinf(curr_wall_side_distances)] = 3.5
            curr_wall_avg_side_distance = np.mean(curr_wall_side_distances)
            # self.get_logger().info('Current average wall distance is %.2f' % curr_wall_avg_side_distance)

            if (curr_wall_avg_side_distance < prev_wall_avg_side_distance):
                    # reset the the average distance of the wall
                    prev_wall_avg_side_distance = curr_wall_avg_side_distance
                    self.get_logger().info('New previous average wall distance is %.2f' % prev_wall_avg_side_distance)

            # check if the obstacle is still on the side
            # if distance suddenly increases significantly, obstacle no longer on the side
            # checked by if the avg distance between previous and current distance from wall defer by more than 50%
            # and the prev_wall_avg_distance needs to be less than 0.5m away to ensure the wall has been detected before
            distance_diff = curr_wall_avg_side_distance - prev_wall_avg_side_distance
            if (distance_diff > (2 * prev_wall_avg_side_distance) and (prev_wall_avg_side_distance <= 2*stop_distance)):
                self.get_logger().info('Side wall no longer detected')
                # update the current distance travelled in the x and y axis
                self.travelled()
                if x_axis == True:
                    curr_pos = self.x_pos
                else:
                    curr_pos = self.y_pos
                # extra distance so that the turtlebot has sufficient space to turn
                # if-case to prevent overshooting initial y pos by accident
                if ((abs(curr_pos - initial_pos) < extra_distance) and moved_off):
                    self.travel_distance(self.curr_dir, abs(curr_pos - initial_pos))
                else:
                    self.travel_distance(self.curr_dir, extra_distance)
                # wall no longer detected, stop and rotate turtlebot
                self.stopbot()

                # checking previous turns and current direction to prevent an endless loop condition
                if (len(bypass_turns) >= 2 and bypass_turns[-2] == bypass_opp_dir
                    and bypass_turns[-1] == bypass_opp_dir and (self.curr_dir == (initial_dir + bypass_opp_dir) % 360)):
                    # reset direction back to initial direction
                    while (self.curr_dir != initial_dir):
                        self.rotatebot(bypass_direction)
                    self.stopbot()
                else:
                    self.rotatebot(bypass_opp_dir)
                    bypass_turns.append(bypass_opp_dir)
                # set moved_off to True only once
                if (moved_off == False):
                    moved_off = True
                    self.get_logger().info('Moved_off set to True')

                # reset the avg side wall distance
                rclpy.spin_once(self)
                self.get_logger().info('Resetting prev average side wall distance')
                prev_wall_avg_side_distance = np.average(self.laser_range[bypass_opp_dir-side_angle:bypass_opp_dir+side_angle])
                self.get_logger().info('New prev average wall distance is %.2f' % prev_wall_avg_side_distance)
                self.move_forward()

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
        # testing if changing target yaw can correct angle
        while (target_yaw > 2*math.pi or target_yaw < -2*math.pi):
            if target_yaw > 2*math.pi:
                target_yaw -= 2*math.pi
            else:
                target_yaw += 2*math.pi

        # self.get_logger().info('Corrected 1 target yaw is %f' % target_yaw)
        if ((target_yaw >= (math.pi - angle_error)) and (target_yaw <= (math.pi + angle_error))):
            target_yaw = math.pi 
        elif ((target_yaw >= (0.5*math.pi) - angle_error) and (target_yaw <= (0.5*math.pi) + angle_error)):
            target_yaw = 0.5*math.pi
        elif ((target_yaw <= -(0.5*math.pi) + angle_error) and (target_yaw >= -(0.5*math.pi) - angle_error)):
            target_yaw = -0.5*math.pi
        elif ((target_yaw >= (1.5*math.pi) - angle_error) and (target_yaw <= (1.5*math.pi) + angle_error)):
            target_yaw = 1.5*math.pi
        else:
            target_yaw = 0.0
        # self.get_logger().info('Corrected 2 target yaw is %f' % target_yaw)

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

    def u_turn_left(self):
        self.get_logger().info('Making a left u-turn')
        # checks if able to turn left
        if np.take(self.laser_range, LEFT) > stop_distance:
            self.get_logger().info('Left u-turn started')

            # rotate left
            self.rotatebot(float(LEFT))

            # start moving
            self.move_forward()
            self.travel_distance(self.curr_dir, 0.30)
            self.stopbot()
            # to update laser_range values in new direction
            rclpy.spin_once(self)

            # if np.take(self.laser_range, LEFT) > stop_distance:
            self.get_logger().info('Completing left u-turn started')
            # rotate left
            self.rotatebot(float(LEFT))
            ### NO LONGER NEEDED
            ## to keep track of current turn
            #if (len(turn_tracker)) != 0:
            #    self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])
            #else:
            #    self.get_logger().info('No previous u-turn')
            #turn_tracker.append('left')
            #self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])

            # reset the x pos
            self.starting_x_pos = self.x_pos
            # start moving
            self.move_forward()
            
    def u_turn_right(self):
        self.get_logger().info('Making a right u-turn')
        # checks if able to turn right
        if np.take(self.laser_range, RIGHT) > stop_distance:
            self.get_logger().info('Right u-turn started')

            # rotate right
            self.rotatebot(float(RIGHT))

            # start moving
            self.move_forward()
            self.travel_distance(self.curr_dir, 0.30)
            self.stopbot()
            # to update laser_range values in new direction
            rclpy.spin_once(self)

            # if np.take(self.laser_range, RIGHT) > stop_distance:
            self.get_logger().info('Completing right u-turn started')
            # rotate right
            self.rotatebot(float(RIGHT))

            ### NO LONGER NEEDED
            ## to keep track of current turn
            #if (len(turn_tracker)) != 0:
            #    self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])
            #else:
            #    self.get_logger().info('No previous u-turn')
            #turn_tracker.append('right')
            #self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])

            # reset the x pos
            self.starting_x_pos = self.x_pos
            # start moving
            self.move_forward()

    def u_turn_back(self):
        self.get_logger().info('Making a rotational u-turn')
        self.rotatebot(float(BACK))
        self.get_logger().info('Finished turning')

        ### NO LONGER NEEDED
        ## to ensure the next turn is correct
        #if turn_tracker[-1] == 'left':
        #    turn_tracker.append('right')
        #else:
        #    turn_tracker.append('left')

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

    ## NO LONGER NEEDED
    #def find_nfc(self):
    #
    #    # to simulate nfc found
    #    #set_time = time.time()
    #    #random.seed(time.time())
    #    #rand_num = random.randint(10, 20)
    #    try:
    #        # initialize variable to write elapsed time to file
    #        # contourCheck = 1
#
    #        # find direction with the largest distance from the Lidar,
    #        # rotate to that direction, and start moving
    #        self.pick_direction()
    #        
    #        while rclpy.ok():
    #            # for simulating nfc found
    #            #curr_time = time.time()
    #            #if (curr_time - set_time) >= rand_num:
    #            #    self.get_logger().info('Simulating NFC found')
    #            #    self.get_logger().info('Exiting NFC func')
    #            #    break
#
    #            self.travelled()
#
    #            if self.laser_range.size != 0:
    #                # check distances in front of TurtleBot and find values less
    #                # than stop_distance
    #                lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
    #                # self.get_logger().info('Distances: %s' % str(lri))
    #                # check if the initial position has been set
    #                if self.not_set:
    #                    self.get_logger().info('Setting initial distance')
    #                    self.get_initial_pos()
#
    #                # if the list is not empty
    #                if(len(lri[0])>0):
    #                    # stop moving
    #                    self.stopbot()
    #                    self.get_logger().info('Obstacle encountered infront')
    #                    if self.edge_reached():
    #                        self.get_logger().info('Reached the edge of the maze')
    #                        # U-turn the turtlebot in the correct direction
    #                        # checks if wall is on the right side for the very first turn. If it is, start with left u-turn
    #                        if (len(turn_tracker) == 0):
    #                            self.get_logger().info('Checking the first turn')
    #                            self.get_logger().info("Left dis: %f, Right dis: %f" % (np.take(self.laser_range, LEFT),np.take(self.laser_range, RIGHT)))
    #                            if np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT):
    #                                self.u_turn_left()
    #                            else:
    #                                self.u_turn_right()
    #                        else:
    #                            self.get_logger().info('Checking subsequent turns')
    #                            # for checking the subsequent turns
    #                            if turn_tracker[-1] == 'left' and np.take(self.laser_range, RIGHT) > stop_distance:
    #                                self.u_turn_right()
    #                            elif turn_tracker[-1] == 'right' and np.take(self.laser_range, LEFT) > stop_distance:
    #                                self.u_turn_left()
    #                            else:
    #                                self.u_turn_back()
    #                    else:
    #                        self.bypass_obstacle()
    #                        self.get_logger().info('Finished bypassing obstacle')
#
    #                        # start moving forward after bypassing
    #                        self.move_forward()
#
    #            # allow the callback functions to run
    #            rclpy.spin_once(self)
#
    #    except Exception as e:
    #        print(e)
    #    
    #    # Ctrl-c detected
    #    finally:
    #        # stop moving
    #        self.stopbot()


    ### CHANGE TO FIT NEW ALGO
    # to move the turtlebot to the edge of the maze before starting algo to find thermal object
    def move_to_edge(self):
        # update the total distance travelled so far
        self.travelled()
        # to determine where the closest wall is for the turtlebot to move to
        if self.x_travelled_dist < self.y_travelled_dist:
            if self.x_travelled_dist < 0.5*self.size:
                wanted_direction = FRONT
            else:
                wanted_direction = BACK
        else:
            if self.y_travelled_dist < 0.5*self.size:
                wanted_direction = LEFT
            else:
                wanted_direction = RIGHT

        ### OLD CODE
        #if len(turn_tracker) == 0:
        #    wanted_direction = FRONT
        #elif turn_tracker[-1] == 'left':
        #    wanted_direction = FRONT
        #else:
        #    wanted_direction = BACK
        
        # if turtlebot further from the wall it came from, go to opposite wall
        if self.travelled_dist > 0.5*full_width_length:
            # swap wanted_direction to the opposite direction
            if wanted_direction == FRONT:
                wanted_direction = BACK
            else:
                wanted_direction = FRONT

        #rotate turtlebot till it is in the correct direction
        ## could be improved for non random direction
        self.get_logger().info("Wanted direction: %i" % wanted_direction)
        while (self.curr_dir != wanted_direction):
            self.rotatebot(LEFT)

        # start moving towards wall
        self.move_forward()

        # allowance of 3cm given
        while (self.travelled_dist > 0.03 or self.travelled_dist < full_width_length - 0.03):
            self.get_logger().info('Current travel distance is %.4f' % self.travelled_dist)
            rclpy.spin_once(self)

            if self.laser_range.size != 0:
                # check distances in front of TurtleBot and find values less
                # than stop_distance
                lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
            if(len(lri[0])>0):
                # stop moving
                self.stopbot()
                self.get_logger().info('Obstacle encountered infront')
                self.travelled()
                if (self.travelled_dist <= 0.03 or self.travelled_dist >= full_width_length - 0.03):
                    self.get_logger().info('Edge of maze reached')
                    break
                # random direction choosen for now
                self.bypass_obstacle()
                self.move_forward()
            # to update the total distance travelled
            self.travelled()
        
        self.stopbot()
        if (np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT)):
            self.rotatebot(LEFT)
        else:
            self.rotatebot(RIGHT)
        self.get_logger().info('Stopping move_to_edge func')

    #def find_thermal(self):
    #    self.get_logger().info('Starting thermal func')
    #    try:
    #        # initialize variable to write elapsed time to file
    #        # contourCheck = 1
#
    #        # find direction with the largest distance from the Lidar,
    #        # rotate to that direction, and start moving
    #        if self.travelled_dist != full_width_length:
    #            self.move_to_edge()
    #        self.get_logger().info('Moved to edge successfully')
#
    #        # update laser range based on new position
    #        rclpy.spin_once(self)
    #        if (np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT)):
    #            wall_location = RIGHT
    #            rot_dir = LEFT
    #            self.get_logger().info('Wall on right side')
    #        else:
    #            wall_location = LEFT
    #            rot_dir = RIGHT
    #            self.get_logger().info('Wall on left side')
    #        
    #        min_left = left_side = np.average(self.laser_range[LEFT-5:LEFT+5])
#
    #        while rclpy.ok():
    #            self.get_logger().info('in thermal loop')
    #            self.travelled()
    #            left_side = np.average(self.laser_range[LEFT-5:LEFT+5])
    #            front = self.laser_range[0]
    #            if (left_side < min_left):
    #                min_left = left_side
    #            self.get_logger().info('Min distance of left wall is %f' % min_left)
    #            self.get_logger().info('Current distance of left wall is %f' % left_side)
#
    #            if self.laser_range.size != 0:
    #                # check distances in front of TurtleBot and find values less
    #                # than stop_distance
    #                if (left_side > 1.5*min_left):
    #                    self.travel_distance(self.curr_dir, 0.1)
    #                    self.rotatebot(LEFT)
    #                    self.move_forward()
    #                    self.get_logger().info("Moving to real edge of maze") 
    #                # self.get_logger().info('Distances: %s' % str(lri))
#
    #                # continue moving forward when there is nothing blocking in front
    #                if (front > stop_distance):
    #                    self.get_logger().info("Distance in front is %f" % front)
    #                    self.get_logger().info("Moving forward to new area")
    #                    self.move_forward()
    #                    self.travel_distance(self.curr_dir, 0.5)
    #                    self.rotatebot(rot_dir)
    #                    # Insert code for checking whether thermal cam detect thermal object here
    #                    self.rotatebot(wall_location)
    #                # if there is a wall in front
    #                else:
    #                    # stop moving
    #                    self.stopbot()
    #                    self.get_logger().info('Obstacle encountered infront')
    #                    self.rotatebot(RIGHT)
#
    #            # allow the callback functions to run
    #            rclpy.spin_once(self)
#
    #    except Exception as e:
    #        print(e)
    #    
    #    # Ctrl-c detected
    #    finally:
    #        # stop moving
    #        self.stopbot()

    # bot travels along the midline of the maze in both x and y axis to complete mapping of maze
    def travel_midline(self):

        # to keep track of which axis it has completed and when it is done
        num_u_turns = 0
        midline_reached = False
        self.get_logger().info("current y-travlled dist is %f" % self.y_travelled_dist)

        try:
            while rclpy.ok():
                self.travelled()
                if self.nfcfound == True:
                    self.stopbot()
                    self.get_logger().info("NFC found")

                # once 1 u turn done, no longer need to stop at the midpoint
                if (midline_reached == False and self.x_travelled_dist >= 0.5*self.size):
                    if np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT):
                        self.rotatebot(LEFT)
                        turn_dir = LEFT
                        self.move_forward()
                    else:
                        self.rotatebot(RIGHT)
                        turn_dir = RIGHT
                        self.move_forward()
                    
                    midline_reached = True
                    self.get_logger().info("Midline reached, moving along down axis")

                # for moving to side of maze to prep for finding nfc and thermal if not yet found
                elif (num_u_turns != 0 and self.y_travelled_dist >= 0.5 * self.size):
                    self.stopbot()
                    self.get_logger().info("Moving to side to prep for finding nfc / thermal")
                    self.rotatebot(turn_dir)
                    self.starting_y_pos = self.y_pos
                    self.move_forward()

                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        self.get_logger().info('Obstacle encountered infront')

                        # check whether to do u turn or exit function
                        if num_u_turns == 0:
                            if midline_reached and self.y_travelled_dist >= 0.5 * self.size:
                                self.get_logger().info('Making u turn')
                                self.u_turn_back()
                                # updating variables
                                num_u_turns += 1
                                self.starting_y_pos = self.y_pos
                            else:
                                self.bypass_obstacle()
                                self.get_logger().info("Obs bypassed, continue moving to midline")

                            self.move_forward()
                        else:
                            # if exiting function, start finding obj in a decrement of 30cm each round around the maze
                            if self.x_travelled_dist >= self.size - 0.3:
                                self.get_logger().info("Exiting function to find obj")
                                self.rotatebot(turn_dir)
                                break
                            # not close enough to wall, move closer
                            else:
                                self.get_logger().info("Moving closer to wall before exiting")
                                self.bypass_obstacle()
                                self.move_forward()

                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()       

    def complete_maze(self):

        # to keep track of whether bot has travelled one full round
        num_turns = 0

        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1
            # allow starting variables to be initialised properly
            rclpy.spin_once(self)
            self.get_logger().info('Setting initial distance')
            self.get_initial_pos()
            # Move forward once it starts
            self.move_forward()
            # for keeping track of size of maze
            size_set = False
            
            while rclpy.ok():

                self.travelled()
                if self.nfcfound == True:
                    self.stopbot()
                    self.get_logger().info("NFC found")

                # travel along mid line once one round completed
                if num_turns == 4:
                    self.get_logger().info("One round completed, travelling to midline")
                    self.travel_midline()
                    
                    # break out of complete_maze function once entire maze is mapped
                    break

                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        if size_set == False:
                            # setting the dimensions of the maze
                            self.size = self.x_travelled_dist
                            size_set = True
                        self.get_logger().info('Reached the edge of the maze')
                        # checks for side with larger clearance and turns there
                        self.get_logger().info('Checking the side clearance')
                        self.get_logger().info("Left dis: %f, Right dis: %f" % (np.take(self.laser_range, LEFT),np.take(self.laser_range, RIGHT)))
                        if np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT):
                            self.rotatebot(LEFT)
                        else:
                            self.rotatebot(RIGHT)
                        self.get_initial_pos()
                        num_turns += 1
                        self.move_forward()

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
    auto_nav.complete_maze()
    auto_nav.load_balls()
    auto_nav.find_objects()
    #auto_nav.adjust_bot()
    #auto_nav.launcher()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
