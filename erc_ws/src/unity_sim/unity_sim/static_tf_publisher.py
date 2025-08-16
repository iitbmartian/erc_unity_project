import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import math

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')

        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.publisher = self.create_publisher(TFMessage, '/tf_static', qos_profile)
        self.publish_transforms()

    def publish_transforms(self):
        transforms = []

        def make_tf(parent, child, t, r):
            tf = TransformStamped()
            tf.header.stamp.sec = 0
            tf.header.stamp.nanosec = 0
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.translation = Vector3(x=t[0], y=t[1], z=t[2])
            tf.transform.rotation = Quaternion(x=r[0], y=r[1], z=r[2], w=r[3])
            return tf

        def euler_to_quaternion(roll, pitch, yaw):
            """Convert Euler angles to quaternion (x, y, z, w)"""
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy

            return [x, y, z, w]

        transforms.append(make_tf('panther/base_link', 'body_link', [0.0, 0.0, 0.2635], [0.0, 0.0, 0.0, 1.0]))
        transforms.append(make_tf('panther/base_link', 'panther/imu_link', [0.0, 0.0, 0.3235], [0.0, 0.0, 0.0, 1.0]))
        transforms.append(make_tf('panther/base_link', 'laser_link', [0.0, 0.0, 0.5435], [0.0, 0.0, 0.0, 1.0]))

        camera_distance = 0.207 
        camera_height = 0.4567

        transforms.append(make_tf('panther/base_link', 'panther/camera_front', 
                                [camera_distance, 0.0, camera_height], 
                                [0.0, 0.0, 0.0, 1.0]))

        transforms.append(make_tf('panther/base_link', 'panther/camera_back', 
                                [-camera_distance, 0.0, camera_height], 
                                euler_to_quaternion(0.0, 0.0, math.pi)))

        transforms.append(make_tf('panther/base_link', 'panther/camera_left', 
                                [0.0, camera_distance, camera_height], 
                                euler_to_quaternion(0.0, 0.0, math.pi/2)))

        transforms.append(make_tf('panther/base_link', 'panther/camera_right', 
                                [0.0, -camera_distance, camera_height], 
                                euler_to_quaternion(0.0, 0.0, -math.pi/2)))

        transforms.append(make_tf('panther/camera_front', 'front_camera_link', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        transforms.append(make_tf('panther/camera_back', 'back_camera_link', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        transforms.append(make_tf('panther/camera_left', 'left_camera_link', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        transforms.append(make_tf('panther/camera_right', 'right_camera_link', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))

        optical_rotation = [-0.5, 0.5, -0.5, 0.5]
        
        transforms.append(make_tf('front_camera_link', 'front_camera_link_optical', [0.0, 0.0, 0.0], optical_rotation))
        transforms.append(make_tf('back_camera_link', 'back_camera_link_optical', [0.0, 0.0, 0.0], optical_rotation))
        transforms.append(make_tf('left_camera_link', 'left_camera_link_optical', [0.0, 0.0, 0.0], optical_rotation))
        transforms.append(make_tf('right_camera_link', 'right_camera_link_optical', [0.0, 0.0, 0.0], optical_rotation))

        msg = TFMessage(transforms=transforms)
        self.publisher.publish(msg)
        self.get_logger().info('Published static TFs for 4 cameras to /tf_static')

def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
