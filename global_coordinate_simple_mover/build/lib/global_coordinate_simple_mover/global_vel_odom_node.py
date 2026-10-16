import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class GlobalVelocityOdomNode(Node):
    def __init__(self):
        super().__init__('global_velocity_odom_node')
        
        # Subscribe ke topik /odom dari Gazebo Harmonic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info('Node Global Velocity (Odom) aktif. Menunggu data /odom...')

    def quaternion_to_yaw(self, q):
        """Mengonversi orientasi Quaternion (x, y, z, w) ke sudut Euler Yaw (radian)."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        # 1. Ambil sudut orientasi robot di Global Coordinate (Yaw / Theta)
        q = msg.pose.pose.orientation
        theta = self.quaternion_to_yaw(q)

        # 2. Ambil kecepatan robot di Local Coordinate (Robot Frame)
        v_local_x = msg.twist.twist.linear.x
        v_local_y = msg.twist.twist.linear.y
        omega_z = msg.twist.twist.angular.z

        # 3. Transformasi kecepatan Local ke Global Coordinate
        v_global_x = v_local_x * math.cos(theta) - v_local_y * math.sin(theta)
        v_global_y = v_local_x * math.sin(theta) + v_local_y * math.cos(theta)

        # 4. Tampilkan hasil di terminal
        theta_deg = math.degrees(theta)
        self.get_logger().info(
            f'Global Vel -> Vx: {v_global_x:6.2f} m/s | Vy: {v_global_y:6.2f} m/s | '
            f'Wz: {omega_z:5.2f} rad/s | Heading (Yaw): {theta_deg:6.1f}°'
        )

def main(args=None):
    rclpy.init(args=args)
    node = GlobalVelocityOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
