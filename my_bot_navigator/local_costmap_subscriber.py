import rclpy
import numpy as np
import sys
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import Twist
from PIL import Image as im
import rclpy.duration
import tf2_ros
import tf2_geometry_msgs

from rclpy.node import Node


class local_subscriber(Node):
    def __init__(self):
        super().__init__('local_subscriber')
        self.sub_costmap = self.create_subscription(
            Costmap,
            '/local_costmap/costmap_raw',
            self.sub_costmap_callback,
            20
        )
        self.sub_cmdvel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.sub_cmdvel_callback,
            20
        )
        self.pub_cmdvel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.costmap = None
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self)
        self.cmdvel = Twist()
        self.forced = False

    def sub_costmap_callback(self, msg):
        costmap = np.array(msg.data)
        self.costmap = costmap.reshape(msg.metadata.size_x, msg.metadata.size_y).T
        self.costmap_grad_x, self.costmap_grad_y = np.gradient(self.costmap)
        transform = self.tf2_buffer.lookup_transform(
            'map',
            'base_link',
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=0.2)
        )
        x_local = int((transform.transform.translation.x - msg.metadata.origin.position.x)*20)
        y_local = int((transform.transform.translation.y - msg.metadata.origin.position.y)*20)
        x = self.costmap_grad_x[x_local, y_local]
        y = self.costmap_grad_y[x_local, y_local]
        quaternion = transform.transform.rotation
        roll, pitch, yaw = self.euler_from_quaternion(quaternion)
        self.robot_grad_x = x * np.cos(yaw) + y * np.sin(yaw)
        self.robot_grad_y = -x * np.sin(yaw) + y * np.cos(yaw)
        self.robot_grad = np.array([self.robot_grad_x, self.robot_grad_y])
        data = im.fromarray(self.costmap, 'L')
        data.save('test.png')
        # np.set_printoptions(threshold=sys.maxsize)
        # self.get_logger().info(str(costmap))
        
        # quaternion = transform.transform.rotation
        # roll, pitch, yaw = self.euler_from_quaternion(quaternion)
        # x = self.cmdvel.linear.x * np.cos(yaw) - self.cmdvel.linear.y * np.sin(yaw)
        # y = self.cmdvel.linear.x * np.sin(yaw) + self.cmdvel.linear.y * np.cos(yaw)
        # z = self.cmdvel.linear.z  # Assuming no height change

        # twist_trans = np.array([x, y, z])
        twist_trans = np.array([self.cmdvel.linear.x, self.cmdvel.linear.y, self.cmdvel.linear.z])
        twist_trans_2d = twist_trans[0:2]
        self.get_logger().info(str(twist_trans_2d) + ' ' + str(self.robot_grad) + ' ' + str(x_local) + str(y_local) + ' ' + str(self.forced))
        if np.dot(twist_trans_2d, self.robot_grad) > 0:
            
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            if self.robot_grad_y < 0:
                vel.angular.z = 0.5
            else:
                vel.angular.z = -0.5
            self.forced = True
            self.pub_cmdvel.publish(vel)
        if self.forced == True and self.robot_grad_x ** 2 < 10  :
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = 0.0
            self.pub_cmdvel.publish(vel)
            # print('f')
            # self.pub_cmdvel.publish(self.cmdvel)
            self.forced = False
            

    
    def sub_cmdvel_callback(self, msg):
        self.cmdvel = msg
        if self.forced == True:
            self.forced = False

    def euler_from_quaternion(self, quaternion):
        """
        Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw).
        """
        import math
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    local_sub = local_subscriber()
    rclpy.spin(local_sub)
    local_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
