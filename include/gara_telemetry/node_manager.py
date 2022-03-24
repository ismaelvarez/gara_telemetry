import rospy
from sensor_msgs.msg import Image
from gara_messages.msg import Telemetry
from robotnik_msgs .msg import BatteryStatus
from nav_msgs.msg import Odometry


class GaraTelemetry:
    def __init__(self):
        # Create a ROS publisher
        self.telemetry_publisher = rospy.Publisher("/robot/GARA/telemetry", Telemetry, queue_size=1)
        # Initialize telemetry data
        self.telemetry = Telemetry()

    def get_telemetry(self):
        self.get_odometry()
        self.get_battery_status()

    def get_odometry(self):
        try:
            odom = rospy.wait_for_message('robot/robotnik_base_control/odom', Odometry, timeout=1)
            position = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
            pose = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.x, odom.pose.pose.orientation.x,
                    odom.pose.pose.orientation.w]
            self.telemetry.position = position
            self.telemetry.pose = pose
        except rospy.exceptions.ROSException:
            rospy.loginfo(rospy.get_caller_id() + 'Timeout: No Odometry message received')

    def get_battery_status(self):
        try:
            battery = rospy.wait_for_message('robot/robotnik_base_control/BatteryStatus', BatteryStatus, timeout=1)
            self.telemetry.battery_level = battery.level
            self.telemetry.battery_time_remaining = battery.time_remaining
            self.telemetry.is_charging = bool(battery.is_charging)
        except rospy.exceptions.ROSException:
            rospy.loginfo(rospy.get_caller_id() + 'Timeout: No Battery message received')

    def publish(self):
        self.telemetry_publisher.publish(self.telemetry)
        rospy.loginfo(rospy.get_caller_id() + 'Publish telemetry')


def init():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gara_telemetry', anonymous=True)

    telemetry = GaraTelemetry()
    # Create a rate
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        telemetry.get_telemetry()
        telemetry.publish()
        rate.sleep()

