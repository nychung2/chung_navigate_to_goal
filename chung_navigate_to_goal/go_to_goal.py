import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist, Point

import math
import numpy as np

class GoalController(Node):
    '''
    '''
    def __init__(self):
        super().__init__('goal_controller')

        self.goals = ((1.5, 0.0), (1.5, 1.4), (0.0, 1.4))
        #self.goals = ((0.5, 0.0), (0.5, 0.4), (0.0, 0.4))
        self.curr_index = 0

        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.globalAng = 0

        self.rotate_thr = 10 # rad
        self.linear_thr = 10 # rad

        self.obstacle_subscriber = self.create_subscription(
            Vector3,
            '/obstacles',
            self.obstacle_callback,
            5
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1
        )

        self.move_publisher = self.create_publisher(Twist, '/cmd_vel', 5)

        self.obstacle_subscriber
        self.odom_subscriber
        self.move_publisher

    def odom_callback(self, location):
        self.update_Odometry(location)

    def obstacle_callback(self, vector):
        state = self.get_state(vector)
        if state == 0: # go to goal
            #self.get_logger().info('Currently trying to go to goal')
            self.go_to_goal()
        elif state == 1: # dodge goal
            self.get_logger().info('Currently running away from my obstacle')
            self.dodge_goal(vector)
        else: #reached goal
            self.publish_message((0.0,0.0))

    def get_state(self, vector):
        if self.curr_index > 2:
            return 5
        
        x_goal = abs(self.globalPos.x - self.goals[self.curr_index][0])
        y_goal = abs(self.globalPos.y - self.goals[self.curr_index][1])
        if x_goal < 0.01 and y_goal < 0.01:
            self.curr_index += 1
        
        if vector.x == -1.0:
            return 0
        else:
            return 1

    def go_to_goal(self):
        target = self.goals[self.curr_index]
        my_location = self.globalPos
        my_orientation = self.globalAng
        self.get_logger().info("orientation: " + str(my_orientation))
        dx = target[0] - my_location.x
        dy = target[1] - my_location.y
        target_distance = math.sqrt((dx ** 2) + (dy ** 2))
        target_angle = math.atan2(dy,dx)

        e = target_angle - my_orientation
        self.get_logger().info(str(target_distance) + " " + str(e))
        if abs(e) > 0.0436: # roughly +/- 2.5 degrees
            kpa = 2
            ua = kpa * e
            if ua > 1.5:
                ua = 1.5
            if ua < -1.5:
                ua = -1.5
            ul = 0.0
        elif abs(target_distance) > 0.001: # +/- 0.05m or 8cm 
            kpl = 40
            ul = kpl * target_distance
            if ul > 0.15:
                ul = 0.15
            if ul < - 0.15:
                ul = -0.15
            ua = 0.0
        else:
            ul = 0.0
            ua = 0.0
        response = (ul, ua)
        self.get_logger().info("sending: " + str(response))
        self.publish_message(response)

    def dodge_goal(self, vector): # wall running? idk i think it is lol. 
        angle = vector[1]
        if angle > 0:
            target_angle = math.pi/2
        else: 
            target_angle = -math.pi/2
        
        e = target_angle - self.globalAng
        if abs(e) > 0.0873: # roughly +/- 5 degrees
            kpa = 2
            ua = kpa * e
            if ua > 1.5:
                ua = 1.5
            if ua < -1.5:
                ua = -1.5
            ul = 0.0
        else:
            if e > 0.0174: # 1 degree
                kpa = 2
                ua = kpa * e
                if ua > 2.0:
                    ua = 2.0
                if ua < -2.0:
                    ua = -2.0
            ul = 0.15
        response = (ua, ul)
        self.publish_message(response)

    def publish_message(self, response):
        msg = Twist()
        msg.linear.x = response[0]
        msg.angular.z = response[1]
        self.move_publisher.publish(msg)

    def update_Odometry(self, Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        if self.globalAng < -math.pi:
            self.globalAng += 2*math.pi
        elif self.globalAng >= math.pi:
            self.globalAng -= 2*math.pi

def main():
    rclpy.init()
    goal_controller = GoalController()
    try:
        rclpy.spin(goal_controller)
    except SystemExit:
        rclpy.get_logger("Goal Controller Node").info("Shutting Down")
    goal_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()