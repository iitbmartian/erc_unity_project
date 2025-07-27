import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

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
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.translation = Vector3(x=t[0], y=t[1], z=t[2])
            tf.transform.rotation = Quaternion(x=r[0], y=r[1], z=r[2], w=r[3])
            return tf

        transforms.append(make_tf('base_link', 'body_link', [0.0, 0.0, 0.2635], [0.0, 0.0, 0.0, -1.0]))
        transforms.append(make_tf('base_link', 'camera_link', [0.2336, 0.06, 0.4567], [0.0, 0.0, 0.0, -1.0]))
        transforms.append(make_tf('camera_link', 'left_camera_link', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, -1.0]))
        transforms.append(make_tf('left_camera_link', 'left_camera_link_optical', [0.0, 0.0, 0.0], [-0.5, 0.5, -0.5, 0.5]))
        transforms.append(make_tf('base_link', 'imu_link', [0.0, 0.0, 0.3235], [0.0, 0.0, 0.0, -1.0]))
        transforms.append(make_tf('base_link', 'lidar_link', [0.0, 0.0, 0.5435], [0.0, 0.0, 0.0, -1.0]))

        msg = TFMessage(transforms=transforms)
        self.publisher.publish(msg)
        self.get_logger().info('Published static TFs to /tf_static')

def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
