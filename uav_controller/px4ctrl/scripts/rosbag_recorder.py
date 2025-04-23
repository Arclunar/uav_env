#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import os
import subprocess
from datetime import datetime
import rospkg

# ANSI escape sequences for colors
class Colors:
    PINK = '\033[35m'
    RESET = '\033[0m'

class RosbagRecorder:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('rosbag_recorder', anonymous=True)

        self.drone_id = rospy.get_param('~drone_id', 0)  # Default to 0 if not set

        # print the drone id
        rospy.loginfo("Drone ID: {}".format(self.drone_id))

        # Create a subscriber to the 'rosbag_control' topic
        self.subscriber = rospy.Subscriber('rosbag_control', Bool, self.control_callback)

        # Topics to record
        self.topics_to_record = [
            # drone_$(arg drone_id)_planning/pos_yaw_cmd
            '/drone_{0}_planning/pos_yaw_cmd'.format(self.drone_id),
            # drone_$(arg drone_id)_planning/pos_cmd
            '/drone_{0}_planning/pos_cmd'.format(self.drone_id),
            # drone_$(arg drone_id)_object_odom_to_planner
            '/drone_{0}_object_odom_to_planner'.format(self.drone_id),
            # drone_<drone_id>_ekf_object_odom_0
            '/drone_{0}_ekf_object_odom_0'.format(self.drone_id),
            # drone_<drone_id>_ekf_object_odom_1
            '/drone_{0}_ekf_object_odom_1'.format(self.drone_id),
            # drone_<drone_id>_optimal_list
            '/drone_{0}_optimal_list'.format(self.drone_id),
            # drone_<drone_id>_global_list
            '/drone_{0}_global_list'.format(self.drone_id),
            '/drone_{0}_planning/trajectory'.format(self.drone_id),
            '/goal',
            '/mavros/local_position/pose',
            '/mavros/global_position/local',
            '/mavros/px4flow/ground_distance',
            'drone_{0}_planning/pos_cmd'.format(self.drone_id),
            'drone_{0}_planning/pos_yaw_cmd'.format(self.drone_id),
            'drone_{0}_track_manager/new_target_id'.format(self.drone_id),
            '/drone_{0}_object_odom_to_planner'.format(self.drone_id),
            '/vrpn_client_node/coax0{0}/pose'.format(self.drone_id),
            '/vrpn_client_node/coax0{0}/twist'.format(self.drone_id),
            '/others_odom',
            '/micolink_tof',
            '/pose_height_fusion',
            '/mavros/imu/data',
            '/mavros/imu/data_raw',
            '/ekf/ekf_odom_corrected',
            '/ekf/ekf_odom',
            '/ekf/ekf_odom_filtered',
            '/ekf/acc_filtered',
            '/ekf/simple_odom',
            '/ekf/test_freq',
            '/px4ctrlcoax/test_freq',
            '/debugPx4ctrl',
            '/mavros/setpoint_raw/local',
            '/mavros/setpoint_raw/attitude',
            '/mavros/rc/in',
            '/mavros/state',
            '/mavros/battery',
            '/px4ctrl/des_debug',
            '/px4ctrl/fsm_debug'
        ]

        # rospack = rospkg.RosPack()
        # workspace_dir = rospack.get_ros_home()
        # 获取工作空间的上一级目录
        # workspace_parent_dir = os.path.dirname(workspace_dir)

        # 拼接上一级目录中的rosbag文件夹路径
        # self.rosbag_directory = os.path.join(workspace_parent_dir, 'rosbag')

        # Directory to save the rosbags
        self.rosbag_directory = os.path.expanduser('~/airdrop_swarm_ws/713workspace/rosbag')
        if not os.path.exists(self.rosbag_directory):
            os.makedirs(self.rosbag_directory)

        # Rosbag process and filename tracking
        self.rosbag_process = None
        self.rosbag_filename = None

    def control_callback(self, msg):
        if msg.data:
            self.start_recording()
        else:
            self.stop_recording()

    def start_recording(self):
        if self.rosbag_process is not None:
            rospy.logwarn("Rosbag recording is already running.")
            return

        # Generate a unique filename
        self.rosbag_filename = self.get_unique_filename()

        # Command to start recording
        command = ['rosbag', 'record', '--tcpnodelay','-O', self.rosbag_filename] + self.topics_to_record
        rospy.loginfo("{}Starting rosbag recording: {}{}".format(Colors.PINK, command, Colors.RESET))
        self.rosbag_process = subprocess.Popen(command)

    def stop_recording(self):
        if self.rosbag_process is None:
            rospy.logwarn("Rosbag recording is not running.")
            return

        rospy.loginfo("{}Stopping rosbag recording: {}{}".format(Colors.PINK, self.rosbag_filename, Colors.RESET))
        self.rosbag_process.terminate()
        self.rosbag_process = None

    def get_unique_filename(self):
        base_filename = os.path.join(self.rosbag_directory, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        index = 1
        filename = base_filename
        while os.path.exists(filename + '.bag'):
            filename = "{}_{}".format(base_filename, index)
            index += 1
        return filename

if __name__ == '__main__':
    try:
        rosbag_recorder = RosbagRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
