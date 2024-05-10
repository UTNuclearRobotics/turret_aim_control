import numpy as np
import math
import rospy
import tf

from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import RobotInfo
from apriltag_ros.msg import AprilTagDetectionArray

from turret_aim_control_interfaces.srv import AimEnable, AimEnableResponse

TURRENT_INFO_SERVICE_NAME = "/pxxls/get_robot_info"
PTU_COMMAND_TOPIC = "/pxxls/commands/joint_group"
BASE_FRAME_ID = "/pxxls/base_link"
TILT_TF_FRAME_ID = "/pxxls/tilt_link"
MAP_FRAME_ID = "/map"
TAG_DETECTION_TOPIC = "/tag_detections"
RATE = 25

DEBUG = False


class TurretControllerService(object):
    def __init__(self):
        rospy.init_node("turret_controller")  # Initialize the ROS node with a name

        # Interacting with the PTU
        # Set the joint_command publisher
        self.joint_commands = JointGroupCommand("turret", [0.0, 0.0])
        self.pub_cmds = rospy.Publisher(
            PTU_COMMAND_TOPIC, JointGroupCommand, queue_size=1
        )

        rospy.wait_for_service(TURRENT_INFO_SERVICE_NAME)
        self.srv_turret_info = rospy.ServiceProxy(TURRENT_INFO_SERVICE_NAME, RobotInfo)
        self.turret_info = self.srv_turret_info("group", "turret")
        self.pan_limits = [
            self.turret_info.joint_lower_limits[0],
            self.turret_info.joint_upper_limits[0],
        ]
        self.tilt_limits = [
            self.turret_info.joint_lower_limits[1],
            self.turret_info.joint_upper_limits[1],
        ]

        # Define your service
        self.service = rospy.Service("aim_enable", AimEnable, self.service_cb)
        self.initialized = False
        self.aim_enable = False
        self.target_frame_id = None

        # Set up tf listener
        self._tf_listener = tf.TransformListener()

        # Set up target detection
        self.detection_pose = None
        DETECTION_TOPIC_NAME = rospy.get_param("detection_topic_name", "tag_detections")
        self._apriltag_sub = rospy.Subscriber(
            DETECTION_TOPIC_NAME, AprilTagDetectionArray, self._apriltag_detection_cb
        )

    def _apriltag_detection_cb(self, msg):
        if msg.detections != []:
            self.detection_pose = True
        else:
            self.detection_pose = False
        return

    def service_cb(self, request):
        # Implement your service callback here
        # This function will be called whenever the service is requested
        # You can process the request and return a response
        self.aim_enable = request.aim_enable
        self.target_frame_id = request.target_frame_id

        # Check if the target is in the frame
        response = AimEnableResponse()
        response.success = self.aim_enable
        response.message = self.target_frame_id
        return response

    def go_home(self):
        (trans_ptu, rot_ptu) = self.lookup_transform(TILT_TF_FRAME_ID)
        _, tilt_theta, pan_theta = tf.transformations.euler_from_quaternion(rot_ptu)
        while (
            not rospy.is_shutdown()
            and np.abs(pan_theta) > 0.01
            and np.abs(tilt_theta) > 0.01
        ):
            (trans_ptu, rot_ptu) = self.lookup_transform(TILT_TF_FRAME_ID)
            _, tilt_theta, pan_theta = tf.transformations.euler_from_quaternion(rot_ptu)
            self.joint_commands.cmd = [0.0, 0.0]
            self.pub_cmds.publish(self.joint_commands)
        return

    def lookup_transform(self, destintation_frame_id, base_frame_id=BASE_FRAME_ID):
        try:
            (trans, rot) = self._tf_listener.lookupTransform(
                base_frame_id, destintation_frame_id, rospy.Time(0)
            )
        except Exception:
            (trans, rot) = (None, None)

        return trans, rot

    def track_target(self, event=None):
        (trans_ptu, rot_ptu) = self.lookup_transform(TILT_TF_FRAME_ID)
        (trans_target, rot_target) = self.lookup_transform(
            destintation_frame_id=self.target_frame_id, base_frame_id=MAP_FRAME_ID
        )

        if self.aim_enable and not self.initialized:
            self.go_home()
            self.initialized = True
            rospy.loginfo("Tracking Initialized")
        elif (
            # not rospy.is_shutdown()
            self.aim_enable
            and self.detection_pose
            and trans_target is not None
            and rot_target is not None
            and trans_ptu is not None
            and rot_ptu is not None
        ):
            x_target = trans_target[0]
            y_target = trans_target[1]
            z_target = trans_target[2]
            rospy.loginfo("X detection: %f", x_target) if DEBUG else None
            rospy.loginfo("Y detection: %f", y_target) if DEBUG else None
            rospy.loginfo("Z detection: %f", z_target) if DEBUG else None

            _, tilt_theta, pan_theta = tf.transformations.euler_from_quaternion(rot_ptu)

            # Calculate corrections
            pan_error = math.atan(x_target / z_target)
            pan_cmd = pan_theta - pan_error

            # assumes negative angle tilts down
            tilt_error = math.atan(y_target / z_target)
            tilt_cmd = tilt_theta - tilt_error
            (
                rospy.loginfo("Pan Error: %f Tilt Error: %f", pan_error, tilt_error)
                if DEBUG
                else None
            )
            rospy.loginfo("Pan: %f Tilt: %f", pan_error, tilt_cmd) if DEBUG else None

            # Publish to PTU
            self.joint_commands.cmd[0] = pan_cmd
            self.joint_commands.cmd[1] = tilt_cmd

            # bound the pan and tilt commands
            self.joint_commands.cmd[0] = np.clip(
                self.joint_commands.cmd[0], self.pan_limits[0], self.pan_limits[1]
            )
            self.joint_commands.cmd[1] = np.clip(
                self.joint_commands.cmd[1], self.tilt_limits[0], self.tilt_limits[1]
            )
            self.pub_cmds.publish(self.joint_commands)

        else:
            # Publish to PTU
            self.joint_commands.cmd[0] = 0
            self.joint_commands.cmd[1] = 0
            self.pub_cmds.publish(self.joint_commands)
            rospy.loginfo("No target detected") if DEBUG else None

    def run(self):
        rospy.loginfo("Turret Controller Service is running")
        rospy.Timer(rospy.Duration(1.0 / RATE), self.track_target)
        rospy.spin()


if __name__ == "__main__":
    node = TurretControllerService()
    node.run()
