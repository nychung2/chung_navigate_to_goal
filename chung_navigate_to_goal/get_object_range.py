import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import math
import numpy as np

class ObjectRange(Node):
    '''
    This node finds obstacles in its way. It should retrieve lidar data,
    then segment out the angles of interest to the robot. The angles of 
    interest are +/- 90 degrees and should look publish a vector of the 
    closest object. The closest object should be no longer than 0.5m

    Note: lidar array will be segmented as 0 rad is dead on, so you need
    to split the data and then find the objects. Also, maybe need to apply
    an averaging filter to remove random distance noise for one range? 
    '''
    def __init__(self):
        super().__init__('object_range')

        lidar_qos_profile = QoSProfile(depth=5)
        lidar_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        lidar_qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        lidar_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile=lidar_qos_profile
        )
        
        self.obstacle_publisher = self.create_publisher(Vector3, '/obstacles', 5)

        self.lidar_subscriber
        self.obstacle_publisher

    def lidar_callback(self, scan_data):
        vector = self.detect_obstacles(scan_data)
        self.publish_message(vector)

    def detect_obstacles(self, lidar_data):
        # angle window of interest: [something rad, something rad]
        target_range = [0, 2*math.pi]

        angle_min = lidar_data.angle_min
        angle_max = lidar_data.angle_max
        angle_inc = lidar_data.angle_increment
        index_range = [int((target_range[0] - angle_min) / angle_inc), int((angle_max - target_range[1]) / angle_inc)]
        distance_data = np.array(lidar_data.ranges)
        l_side = distance_data[0:0]
        r_side = distance_data[0:0]
        return None

    def publish_message(self, vector):
        msg = Vector3()
        if vector == None:
            msg.x = -1
        else:
            msg.x = vector[0]
            msg.y = vector[1]
        self.obstacle_publisher.publish(msg)

def main():
    rclpy.init()
    object_range = ObjectRange()
    try:
        rclpy.spin(object_range)
    except SystemExit:
        rclpy.get_logger("Object Range Node").info("Shutting Down")
    object_range.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        

    