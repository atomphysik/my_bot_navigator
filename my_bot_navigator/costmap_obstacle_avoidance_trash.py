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
        self.prev_cmdvel = Twist()
        self.forced = False
        self.timer = None

    def sub_costmap_callback(self, msg):
        """
        Costmap gradient and its dot product with robot velocity is calculated.
        If the value of dot product is positive, robot comes into the forced mode
        until the direction is adjusted.
        After adjustment, it follows the previous velocity again.
        """
        # Local costmap is being saved.
        costmap = np.array(msg.data)
        self.costmap = costmap.reshape(msg.metadata.size_x, msg.metadata.size_y).T
        # Calculated the gradient of costmap.
        self.costmap_grad_x, self.costmap_grad_y = np.gradient(self.costmap)
        # Transform object for coordinate transformation of gradient vector.  
        transform = self.tf2_buffer.lookup_transform(
            'odom',
            'base_link',
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=0.2)
        )
        # Roll, pitch, yaw 
        quaternion = transform.transform.rotation
        roll, pitch, yaw = self.euler_from_quaternion(quaternion)
        # The position of robot in the local costmap grid.
        x_local = int((transform.transform.translation.x - msg.metadata.origin.position.x )*20)
        y_local = int((transform.transform.translation.y - msg.metadata.origin.position.y)*20)
        
    
        if self.cmdvel != None:
            #cmd_vel in odometry reference frame
            odom_cmdvel_x = self.cmdvel.linear.x * np.cos(yaw) - self.cmdvel.linear.y * np.sin(yaw)
            odom_cmdvel_y = self.cmdvel.linear.x * np.sin(yaw) + self.cmdvel.linear.y * np.cos(yaw)
            odom_cmdvel_z = self.cmdvel.linear.z  # Assuming no height change
            # x and y shift in grid for early detection of obstacle.
            x_shift = int((odom_cmdvel_x) * 0.7 * 20)
            y_shift = int((odom_cmdvel_y) * 0.7 * 20)

        # Calculate the gradient vector of costmap at the position of robot.
        x_lower = min(x_local, x_local + x_shift)
        y_lower = min(y_local, y_local + y_shift)
        grad_x = self.costmap_grad_x[x_lower : x_lower + abs(x_shift) + 1, y_lower : y_lower + abs(y_shift) + 1]
        grad_y = self.costmap_grad_y[x_lower : x_lower + abs(x_shift) + 1, y_lower : y_lower + abs(y_shift) + 1]
        self.robot_grad_x = grad_x * np.cos(yaw) + grad_y * np.sin(yaw)
        self.robot_grad_y = -grad_x * np.sin(yaw) + grad_y * np.cos(yaw)
        if x_shift == 0 and y_shift == 0:
            self.robot_grad = np.array([self.robot_grad_x[0], self.robot_grad_y[0]])
        else:
            self.robot_grad_x = self.robot_grad_x.ravel()
            self.robot_grad_y = self.robot_grad_y.ravel()
            self.robot_grad = np.array([self.robot_grad_x, self.robot_grad_y])
            # self.robot_grad = self.robot_grad.T
        # data = im.fromarray(self.costmap, 'L')
        # data.save('test.png')
        
        # np.set_printoptions(threshold=sys.maxsize)
        # self.get_logger().info(str(costmap))
        
        # quaternion = transform.transform.rotation
        # roll, pitch, yaw = self.euler_from_quaternion(quaternion)
        
        # twist_trans = np.array([x, y, z])
        
        # Get the velocity vector of robot.
        twist_trans = np.array([self.cmdvel.linear.x, self.cmdvel.linear.y, self.cmdvel.linear.z])
        twist_trans_2d = twist_trans[0:2]
        self.get_logger().info(str(twist_trans_2d) + ' ' + str(self.robot_grad) + ' ' + str(self.cmdvel.linear.x) +' ' + str(x_shift) + str(y_shift))
        # Determine if gradient vector and velocity vector is positive or negative.
        # If it is positive, we adjust the direction by rotating the robot in the most efficient direction.
        slope = np.dot(twist_trans_2d, self.robot_grad)
        if type(slope) == int and slope > 0 or (slope>0).any():
            
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            if x_shift * y_shift < 0:
                grad_y_pick = self.robot_grad_y[self.robot_grad_y.shape[0] // 2]
            elif x_shift > 0:
                grad_y_pick = self.robot_grad_y[-1]
            else:
                grad_y_pick = self.robot_grad_y[0]
            if  grad_y_pick < 0:
                vel.angular.z = 1.0
            else:
                vel.angular.z = -1.0
            self.prev_cmdvel = self.cmdvel
            self.pub_cmdvel.publish(vel)
            self.timer = self.create_timer(0.2, self.get_forced)
        elif self.forced == True and self.robot_grad_x ** 2 < 100 :
            # vel = Twist()
            # vel.linear.x = 0.0
            # vel.linear.y = 0.0
            # vel.linear.z = 0.0
            # vel.angular.x = 0.0
            # vel.angular.y = 0.0
            # vel.angular.z = 0.0
            self.pub_cmdvel.publish(self.prev_cmdvel)
            # print('f')
            # self.pub_cmdvel.publish(self.cmdvel)
            self.forced = False
            

    
    def sub_cmdvel_callback(self, msg):
        """
        Save cmd_vel to know the current status of robot.
        """
        self.cmdvel = msg
        if self.forced == True:
            self.forced = False

    def get_forced(self):
        """
        Change robot mode to forced.
        """
        self.forced = True
        self.timer.cancel()

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
