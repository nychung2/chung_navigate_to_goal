import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist, Point

#rom rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import math
import numpy as np
import time

# changed qos, odom callback. 

class GoalController(Node):
    '''
    '''
    def __init__(self):
        super().__init__('goal_controller')

        #self.goals = ((1.5, 0.0), (1.5, 1.4), (0.0, 1.4))
        self.goals = ((1.65, 0,0), (1.65, 1.54), (0.0, 1.54))
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
        self.start = True

        #self.loop_rate = self.create_rate(0.1, self.get_clock())

        self.obstacle_subscriber = self.create_subscription(
            Vector3,
            '/obstacles',
            self.obstacle_callback,
            5
        )

        # #change qos below
        # qos = QoSProfile(depth=5)
        # qos.history = QoSHistoryPolicy.KEEP_LAST
        # qos.durability = QoSDurabilityPolicy.VOLATILE 
        # qos.reliability = QoSReliabilityPolicy.BEST_EFFORT 

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1 #qos
        )

        self.move_publisher = self.create_publisher(Twist, '/cmd_vel', 5)

        self.obstacle_subscriber
        self.odom_subscriber
        self.move_publisher

    def odom_callback(self, location):
        self.update_Odometry(location)
        # delete below only for create 3 tests
        # msg = Vector3()
        # msg.x = -1.0
        # state = self.get_state(msg)
        # self.go_to_goal() # temporary

    def obstacle_callback(self, vector):
        state = self.get_state(vector)
        if state == 0: #or self.curr_index == 0: # go to goal
            #self.get_logger().info('Currently trying to go to goal')
            self.go_to_goal()
        elif state == 1: # dodge goal
            self.get_logger().info('Currently running away from my obstacle')
            self.dodge_goal(vector)
        else: #reached goal
            self.publish_message((0.0,0.0))

    def get_state(self, vector):
        if self.start == True:
            for _ in range(30):
                self.publish_message((0.0,0.0))
            self.start = False

        if self.curr_index > 2:
            return 5
        
        x_goal = abs(self.globalPos.x - self.goals[self.curr_index][0])
        y_goal = abs(self.globalPos.y - self.goals[self.curr_index][1])
        #if x_goal < 0.01 and y_goal < 0.01:
        if math.sqrt(x_goal**2 + y_goal**2) < 0.012:
            self.curr_index += 1
            for _ in range(10):
                self.publish_message((0.0,0.0))
            time.sleep(10)
            #self.loop_rate.sleep()
        if self.curr_index == 3:
            raise SystemExit
        
        if vector.x == -1.0:
            return 0
        else:
            return 1

    def go_to_goal(self):
        self.get_logger().info("-----------------------------------------------------------")
        target = self.goals[self.curr_index]
        my_location = self.globalPos
        my_orientation = self.globalAng
        dx = target[0] - my_location.x
        dy = target[1] - my_location.y
        target_distance = math.sqrt((dx ** 2) + (dy ** 2))
        #target_distance = math.sqrt(target[0] ** 2 + target[1] ** 2) - math.sqrt(my_location.x ** 2 + my_location.y ** 2)
        target_angle = math.atan2(dy,dx)
        self.get_logger().info("target: " + str(target))
        self.get_logger().info("target angle: " + str(target_angle))
        # try this if below doesnt work
        # margin = 0.04
        # if abs(self.globalAng) < math.pi + margin:
        #     if self.globalAng > 0:
        #         if target_angle < 0:
        #             target_angle *= -1
        #     elif self.globalAng < 0:
        #         if target_angle > 0:
        #             target_angle *= -1

        # if self.curr_index == 2:        
        #     epos = target_angle - my_orientation
        #     eneg = -target_angle + my_orientation #-target_angle - my_orientation
        #     arr = np.array([epos, eneg])
        #     self.get_logger().info("error pos: %s, error neg: %s orientation: %s" %(epos * 180 / math.pi, eneg * 180 / math.pi, my_orientation* 180 / math.pi))
        #     e = arr[np.argmin(np.abs(arr))]
        #     target_distance *= -1
        # else:
        #     e = target_angle - my_orientation

        e = target_angle - my_orientation
        if e > math.pi:
            e -= 2*math.pi
        elif e < -math.pi:
            e += 2*math.pi
        self.get_logger().info("target dist: " + str(target_distance) + " angle error: " + str(e*180/math.pi))
        self.get_logger().info("error: %s, " %(str(e),))
        if abs(e) > 0.349: # roughly +/- 20 degrees
            kpa = 2
            ua = kpa * e
            if ua > 1.0:
                ua = 1.0
            if ua < -1.0:
                ua = -1.0
            self.publish_message((0.0, ua))
        if abs(e) > 0.02: # roughly +/- 1 degrees
            kpa = 2
            ua = kpa * e
            if ua > 1.0:
                ua = 1.0
            if ua < -1.0:
                ua = -1.0
            #ul = 0.0
        else:
            ua = 0.0
        if abs(target_distance) > 0.0001: # +/- 0.05m or 8cm 
            kpl = 50
            ul = kpl * abs(target_distance)
            if ul > 0.1:
                ul = 0.1
            if ul < -0.1:
                ul = -0.1
        else:
            ul = 0.0
            #ua = 0.0
        response = (ul, ua)
        #self.get_logger().info("sending: " + str(response))
        self.publish_message(response)

    def dodge_goal(self, vector): # wall running? idk i think it is lol. 
        angle = vector.y
        target_angle = 0.9 #math.pi/2
        # if angle > 0:
        #     target_angle = math.pi/2
        # else:  
        #     target_angle = -1 * math.pi/2
        
        e = target_angle + angle
        self.get_logger().info("angle error: " + str(e))
        if abs(e) > 0.1745: # roughly +/- 10 degrees
            kpa = 2
            ua = kpa * e
            if ua > 1.0:
                ua = 1.0
            if ua < -1.0:
                ua = -1.0
            ul = 0.05
        else:
            if e > 0.0463: # 2.5 degree
                kpa = 2
                ua = kpa * e
                if ua > 1.0:
                    ua = 1.0
                if ua < -1.0:
                    ua = -1.0
            else:
                ua = 0.0
            ul = 0.1
        response = (ul, ua)
        self.get_logger().info("response: " + str(response))
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
        goal_controller.get_logger().info("Shutting Down")
    goal_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()