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

        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()

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
        if vector.x == -1: # no obstacles in way, proceed to goal. 
            response = ()
        else: # there is an obstacle in the way somwhere
            angle = math.atan(vector.x / vector.y)
            if abs(angle) < self.rotate_thr # rotate away
            else: # move straight 
            response = ()
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