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

# constants
rotatechange = 0.2
speedchange = 0.3
obstaclespeed = 0.2
uturnforwardspeed = 0.1
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
turtlebot_length = 0.30
full_width_length = 5.0 - 2*stop_distance - turtlebot_length
angle_error = (2.0/180) * math.pi
front_angle = 20
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
LEFT = 89
RIGHT = 269
BACK = 179


turn_tracker = []

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
        self.x_pos = 0
        self.starting_x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        self.distance_travelled = 0
        self.not_set = True
        
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

    # to set the initial position so that the distance travelled can be tracked
    def get_initial_x_pos(self):
        self.starting_x_pos = self.x_pos
        self.not_set = False
        self.get_logger().info('Starting horizontal distance position = %.2f' % self.starting_x_pos)

    # for measuring the distance travelled 
    def travelled(self):
        self.distance_travelled = abs(self.x_pos - self.starting_x_pos)
        # self.get_logger().info('Total horizontal distance travelled = %.2f' % self.distance_travelled)

    # check if the robot is at the end of the maze
    def edge_reached(self):
        if self.distance_travelled < full_width_length:
            return False
        # reset the initial position
        self.starting_x_pos = self.x_pos
        # reset value of distance travelled
        self.distance_travelled = 0
        return True

    # function to bypass the obstacle to continue finding the NFC
    def bypass_obstacle(self):
        # to prevent situation in which turtlebot stuck in starting point of maze which is a dead end
        # if no u-turn done yet
        if len(turn_tracker) == 0:
            if (np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT)):
                bypass_direction = LEFT
                bypass_opp_dir = RIGHT
            else:
                bypass_direction = RIGHT
                bypass_opp_dir = LEFT 
        # check prev u-turn direction
        if turn_tracker[-1] == 'left':
            bypass_direction = RIGHT
            bypass_opp_dir = LEFT
        else:
            bypass_direction = LEFT
            bypass_opp_dir = RIGHT

        self.odom_subscription 
        # variable used to hold the current y position of the turtlebot
        initial_y = self.y_pos
        self.get_logger().info('Initial y position is: %.2f' % initial_y)
        # turn the turtlebot before starting adopted Pledge algorithm
        self.get_logger().info('Turtlebot turned %s to start Pledge Algo' % bypass_direction)
        self.rotatebot(bypass_direction)
        # setting the first avg wall distance
        prev_wall_avg_side_distance = np.average(self.laser_range[bypass_opp_dir-5:bypass_opp_dir+5])
        # start moving forward after turn is made
        self.get_logger().info('Moving forward')
        twist = Twist()
        twist.linear.x = obstaclespeed
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

        # variable to prevent turtlebot from exiting function even before it starts moving
        moved_off = False

        # Loop to continuously check till turtlebot is done
        # Breaks out of loop automatically when done
        while (True):

            # to update the laser_range values
            rclpy.spin_once(self)

            # check if the turtlebot has reached the other side after moving off, with an allowance of 1 mm
            if (moved_off and (abs(initial_y - self.y_pos) <= 0.01)):
                # turtlebot has successfully navigated around the obstacle, exiting function to resume normal navigation
                self.get_logger().info('Turtlebot successfully navigated around the obstacle')
                self.rotatebot(bypass_direction)
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
                    # to update the variable self.distance_travelled
                    self.travelled()

                    # exit bypass function if edge of maze is reached
                    if (self.distance_travelled >= full_width_length):
                        self.get_logger().info('Reached the edge of the maze while bypassing obs')
                        break
                    if np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT):
                        self.rotatebot(LEFT)
                    else:
                        self.rotatebot(RIGHT)

                    # for moving the turtlebot forward
                    twist = Twist()
                    twist.linear.x = obstaclespeed
                    twist.angular.z = 0.0
                    # not sure if this is really necessary, but things seem to work more
                    # reliably with this
                    time.sleep(1)
                    self.publisher_.publish(twist)

            # calculate the current avg distance from wall from front and side
            # self.get_logger().info('Updating current average wall distance')
            curr_wall_side_distances = self.laser_range[bypass_opp_dir-5:bypass_opp_dir+5]
            curr_wall_avg_side_distance = np.average(curr_wall_side_distances)
            # self.get_logger().info('Current average wall distance is %.2f' % curr_wall_avg_side_distance)

            if (curr_wall_avg_side_distance < prev_wall_avg_side_distance):
                    # reset the the average distance of the wall on the wall
                    prev_wall_avg_side_distance = curr_wall_avg_side_distance
                    self.get_logger().info('New previous average wall distance is %.2f' % prev_wall_avg_side_distance)

            # check if the obstacle is still on the side
            # if distance suddenly increases significantly, obstacle no longer on the side
            # checked by if the avg distance between previous and current distance from wall defer by more than 50%
            # and the prev_wall_avg_distance needs to be less than 0.5m away to ensure the wall has been detected before
            distance_diff = curr_wall_avg_side_distance - prev_wall_avg_side_distance
            if (distance_diff > (2 * prev_wall_avg_side_distance) and (prev_wall_avg_side_distance <= 0.5)):
                # lag time given so that the turtlebot has sufficient space to turn
                time.sleep(0.5)
                # wall no longer detected, rotate turtlebot
                self.get_logger().info('Side wall no longer detected')
                self.stopbot()
                self.rotatebot(bypass_opp_dir)
                # start moving forward after turn is made
                self.get_logger().info('Moving forward')
                # set moved_off to True only once
                if (moved_off == False):
                    moved_off = True
                    self.get_logger().info('Moved_off set to True')

                # reset the avg side wall distance
                self.get_logger().info('Resetting prev average side wall distance')
                prev_wall_avg_side_distance = np.average(self.laser_range[bypass_opp_dir-5:bypass_opp_dir+5])
                self.get_logger().info('New prev average wall distance is %.2f' % prev_wall_avg_side_distance)

                # for moving the turtlebot forward
                twist = Twist()
                twist.linear.x = obstaclespeed
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
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))

        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        self.get_logger().info('Actual target yaw is %f' % target_yaw)
        # testing if changing target yaw can correct angle
        while (target_yaw > 2*math.pi or target_yaw < -2*math.pi):
            if target_yaw > 2*math.pi:
                target_yaw -= 2*math.pi
            else:
                target_yaw += 2*math.pi

        self.get_logger().info('Corrected 1 target yaw is %f' % target_yaw)
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
        self.get_logger().info('Corrected 2 target yaw is %f' % target_yaw)

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
        # set the rotation speed to 0
        #twist.angular.z = 0.0
        # stop the rotation
        #self.publisher_.publish(twist)


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
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

    def u_turn_left(self):
        self.get_logger().info('Making a left u-turn')
        # checks if able to turn left
        if np.take(self.laser_range, LEFT) > stop_distance:
            self.get_logger().info('Left u-turn started')

            # rotate left
            self.rotatebot(float(LEFT+1))

            # start moving
            self.get_logger().info('Moving forward')
            twist = Twist()
            twist.linear.x = uturnforwardspeed
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            time.sleep(1)
            self.publisher_.publish(twist)
            num_times = 3
            while num_times:

                # to update the laser_range values
                rclpy.spin_once(self)

                if np.take(self.laser_range, 0) > stop_distance:
                    time.sleep(1)
                    num_times -= 1
                else:
                    break
            self.stopbot()

            if np.take(self.laser_range, LEFT) > stop_distance:
                rotate_direction = LEFT
                self.get_logger().info('Completing left u-turn started')

                # rotate left
                self.rotatebot(float(LEFT+1))

                # to keep track of current turn
                if (len(turn_tracker)) != 0:
                    self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])
                else:
                    self.get_logger().info('No previous u-turn')

                turn_tracker.append('left')
                self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])

                # reset the x pos
                self.get_initial_x_pos()

                # start moving
                self.get_logger().info('Moving forward')
                twist = Twist()
                twist.linear.x = speedchange
                twist.angular.z = 0.0
                # not sure if this is really necessary, but things seem to work more
                # reliably with this
                time.sleep(1)
                self.publisher_.publish(twist)
            
    def u_turn_right(self):
        self.get_logger().info('Making a right u-turn')
        # checks if able to turn right
        if np.take(self.laser_range, RIGHT) > stop_distance:
            rotate_direction = RIGHT
            self.get_logger().info('Right u-turn started')

            # rotate right
            self.rotatebot(float(RIGHT+1))

            # start moving
            self.get_logger().info('Moving forward')
            twist = Twist()
            twist.linear.x = uturnforwardspeed
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            time.sleep(1)
            self.publisher_.publish(twist)
            num_times = 3
            while num_times:

                # to update the laser_range values
                rclpy.spin_once(self)

                if np.take(self.laser_range, 0) > stop_distance:
                    time.sleep(1)
                    num_times -= 1
                else:
                    break
            self.stopbot()

            if np.take(self.laser_range, RIGHT) > stop_distance:
                rotate_direction = RIGHT
                self.get_logger().info('Completing right u-turn started')

                # rotate right
                self.rotatebot(float(RIGHT+1))

                # to keep track of current turn
                if (len(turn_tracker)) != 0:
                    self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])
                else:
                    self.get_logger().info('No previous u-turn')

                turn_tracker.append('right')
                self.get_logger().info('Previous u-turn: %s' % turn_tracker[-1])

                # reset the x pos
                self.get_initial_x_pos()

                # start moving
                self.get_logger().info('Moving forward')
                twist = Twist()
                twist.linear.x = speedchange
                twist.angular.z = 0.0
                # not sure if this is really necessary, but things seem to work more
                # reliably with this
                time.sleep(1)
                self.publisher_.publish(twist)

    def u_turn_back(self):
        self.get_logger().info('Making a rotational u-turn')
        self.rotatebot(float(90))
        self.rotatebot(float(90))
        self.get_logger().info('Finsished turning')
        # to ensure the next turn is correct
        if turn_tracker[-1] == 'left':
            turn_tracker.append('right')
        else:
            turn_tracker.append('left')
        # start moving
        self.get_logger().info('Moving forward')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def mover(self):
        num_turns = 0
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.pick_direction()
            
            while rclpy.ok():
                self.travelled()

                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))
                    # check if the initial position has been set
                    if self.not_set:
                        self.get_logger().info('Setting initial distance')
                        self.get_initial_x_pos()

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        self.get_logger().info('Obstacle encountered infront')
                        if self.edge_reached():
                            self.get_logger().info('Reached the edge of the maze')
                            # U-turn the turtlebot in the correct direction
                            # checks if wall is on the right side for the very first turn. If it is, start with left u-turn
                            if (len(turn_tracker) == 0):
                                self.get_logger().info('Checking the first turn')
                                if np.take(self.laser_range, LEFT) > np.take(self.laser_range, RIGHT):
                                    self.u_turn_left()
                                else:
                                    self.u_turn_right()
                            else:
                                self.get_logger().info('Checking subsequent turns')
                                # for checking the subsequent turns
                                if turn_tracker[-1] == 'left' and np.take(self.laser_range, RIGHT) > stop_distance:
                                    self.u_turn_right()
                                elif turn_tracker[-1] == 'right' and np.take(self.laser_range, LEFT) > stop_distance:
                                    self.u_turn_left()
                                else:
                                    self.u_turn_back()
                            num_turns += 1
                            self.get_logger().info('Current num of turns: %d' % num_turns)
                        else:
                            self.bypass_obstacle()
                            self.get_logger().info('Finished bypassing obstacle')

                            # start moving forward after bypassing
                            self.get_logger().info('Moving forward')
                            twist = Twist()
                            twist.linear.x = speedchange
                            twist.angular.z = 0.0
                            # not sure if this is really necessary, but things seem to work more
                            # reliably with this
                            time.sleep(1)
                            self.publisher_.publish(twist)
                            

                # allow the callback functions to run
                rclpy.spin_once(self)

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

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
