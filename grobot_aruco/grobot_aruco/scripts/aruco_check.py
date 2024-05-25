# import rclpy
# from rclpy.node import Node
# from tf2_ros import TransformBroadcaster, Buffer, TransformListener
# from geometry_msgs.msg import TransformStamped, Quaternion, PoseWithCovarianceStamped
# import tf2_ros
# from ros2_aruco_interfaces.msg import ArucoMarkers
# from nav_msgs.msg import Odometry

# class ArucoTfPublisher(Node):
#     def __init__(self):
#         super().__init__('aruco_tf_publisher')
#         self.subscription = self.create_subscription(
#             ArucoMarkers,
#             'aruco_markers',
#             self.aruco_marker_callback,
#             10)
#         self.amcl_sub = self.create_subscription(
#             PoseWithCovarianceStamped,
#             'amcl_pose',
#             self.amcl_callback,
#             10)
#         self.tf_broadcaster = TransformBroadcaster(self)
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         self.last_amcl_pose = None
#         self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)

#     def amcl_callback(self, msg):
#         self.last_amcl_pose = msg.pose.pose

#     def aruco_marker_callback(self, msg):
#         if self.last_amcl_pose is None:
#             return

#         marker_positions = []
#         marker_orientations = []
#         for marker_id, pose in zip(msg.marker_ids, msg.poses):
#             t = TransformStamped()
#             t.header.stamp = self.get_clock().now().to_msg()
#             t.header.frame_id = 'map'
#             t.child_frame_id = f"aruco_marker_{marker_id}"
#             t.transform.translation.x = pose.position.x
#             t.transform.translation.y = pose.position.y
#             t.transform.translation.z = pose.position.z
#             t.transform.rotation = pose.orientation
#             self.tf_broadcaster.sendTransform(t)

#             marker_positions.append((pose.position.x, pose.position.y, pose.position.z))
#             marker_orientations.append((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

#         if marker_positions:
#             avg_x = sum([pos[0] for pos in marker_positions]) / len(marker_positions)
#             avg_y = sum([pos[1] for pos in marker_positions]) / len(marker_positions)
#             avg_z = sum([pos[2] for pos in marker_positions]) / len(marker_positions)
#             avg_orientation = marker_orientations[0]

#             corrected_pose = self.combine_poses(self.last_amcl_pose, avg_x, avg_y, avg_z, avg_orientation)

#             self.publish_corrected_pose(corrected_pose)

#     def combine_poses(self, amcl_pose, marker_x, marker_y, marker_z, marker_orientation):
#         corrected_x = (amcl_pose.position.x + marker_x) / 2
#         corrected_y = (amcl_pose.position.y + marker_y) / 2
#         corrected_z = (amcl_pose.position.z + marker_z) / 2

#         corrected_orientation = Quaternion()
#         corrected_orientation.x = marker_orientation[0]
#         corrected_orientation.y = marker_orientation[1]
#         corrected_orientation.z = marker_orientation[2]
#         corrected_orientation.w = marker_orientation[3]

#         corrected_pose = {'x': corrected_x, 'y': corrected_y, 'z': corrected_z, 'orientation': corrected_orientation}
#         return corrected_pose

#     def publish_corrected_pose(self, corrected_pose):
#         pose_msg = PoseWithCovarianceStamped()
#         pose_msg.header.stamp = self.get_clock().now().to_msg()
#         pose_msg.header.frame_id = 'map'  
#         pose_msg.pose.pose.position.x = corrected_pose['x']
#         pose_msg.pose.pose.position.y = corrected_pose['y']
#         pose_msg.pose.pose.position.z = corrected_pose['z']
#         pose_msg.pose.pose.orientation = corrected_pose['orientation']

#         self.pose_publisher.publish(pose_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     aruco_tf_publisher = ArucoTfPublisher()
#     rclpy.spin(aruco_tf_publisher)
#     aruco_tf_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
import tf2_ros
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Quaternion, PoseWithCovarianceStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class ArucoTfPublisher(Node):
    def __init__(self):
        super().__init__('aruco_tf_publisher')
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_amcl_pose = None
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)

    def amcl_callback(self, msg):
        self.last_amcl_pose = msg.pose.pose
        
    def slerp_average_quaternions(quaternions, weights=None):
        if weights is None:
            weights = [1.0] * len(quaternions)
        avg_quaternion = R.from_quat(quaternions[0])
        total_weight = weights[0]
        for q, weight in zip(quaternions[1:], weights[1:]):
            q_rotation = R.from_quat(q)
            avg_quaternion = avg_quaternion.slerp(q_rotation, weight / (total_weight + weight))
            total_weight += weight
        return avg_quaternion.as_quat()

    def aruco_marker_callback(self, msg):
        if self.last_amcl_pose is None:
            return

        marker_positions = []
        marker_orientations = []
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f"aruco_marker_{marker_id}"
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(t)

            marker_positions.append((pose.position.x, pose.position.y, pose.position.z))
            marker_orientations.append((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

        if marker_positions:
            avg_x = sum([pos[0] for pos in marker_positions]) / len(marker_positions)
            avg_y = sum([pos[1] for pos in marker_positions]) / len(marker_positions)
            avg_z = sum([pos[2] for pos in marker_positions]) / len(marker_positions)
            avg_orientation = marker_orientations[0]

            corrected_pose = self.combine_poses(self.last_amcl_pose, avg_x, avg_y, avg_z, avg_orientation)

            self.publish_corrected_pose(corrected_pose)

    def combine_poses(self, amcl_pose, marker_x, marker_y, marker_z, marker_orientation):
        corrected_x = (amcl_pose.position.x + marker_x) / 2
        corrected_y = (amcl_pose.position.y + marker_y) / 2
        corrected_z = (amcl_pose.position.z + marker_z) / 2
        
        # slerp
        amcl_quaternion = (amcl_pose.orientation.x, amcl_pose.orientation.y, amcl_pose.orientation.z, amcl_pose.orientation.w)
        marker_quaternion = marker_orientation
        amcl_rotation = R.from_quat(amcl_quaternion)
        marker_rotation = R.from_quat(marker_quaternion)
        
        # slerp를 사용하여 중간 쿼터니언 계산
        corrected_rotation = amcl_rotation.slerp(marker_rotation, [0.5])[0]
        corrected_quaternion = corrected_rotation.as_quat()

        corrected_orientation = Quaternion()
        corrected_orientation.x = corrected_quaternion[0]
        corrected_orientation.y = corrected_quaternion[1]
        corrected_orientation.z = corrected_quaternion[2]
        corrected_orientation.w = corrected_quaternion[3]

        corrected_pose = Pose()
        corrected_pose.position.x = corrected_x
        corrected_pose.position.y = corrected_y
        corrected_pose.position.z = corrected_z
        corrected_pose.orientation = corrected_orientation
        
        return corrected_pose

    def publish_corrected_pose(self, corrected_pose):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose = corrected_pose

        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    aruco_tf_publisher = ArucoTfPublisher()
    rclpy.spin(aruco_tf_publisher)
    aruco_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
