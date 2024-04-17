#!/usr/bin/env python

import rospy

from turret_aim_control_interfaces.srv import AimEnable, AimEnableResponse

from std_msgs.msg import String  # Import the appropriate message type for publishing

TURRENT_INFO_SERVICE_NAME = "/pxxls/get_robot_info"
PTU_COMMAND_TOPIC = "/pxxls/commands/joint_group"


class TurretControllerService(object):
    def __init__(self):
        rospy.init_node("turret_controller")  # Initialize the ROS node with a name
        # self.command_publisher = rospy.Publisher(
        #     "your_command_topic", String, queue_size=10
        # )  # Publisher to publish commands

        # Define your service
        self.service = rospy.Service("aim_enable", AimEnable, self.service_cb)

    def service_cb(self, request):
        # Implement your service callback here
        # This function will be called whenever the service is requested
        # You can process the request and return a response
        aim_enable = request.aim_enable
        frame_id = request.target_frame_id

        response = AimEnableResponse()
        response.success = True
        response.message = frame_id
        return response

    # def publish_command(self, command):
    #     # Publish command to the topic
    #     self.command_publisher.publish(command)

    def run(self):
        # ROS node main loop
        rospy.spin()


if __name__ == "__main__":
    node = TurretControllerService()
    node.run()
