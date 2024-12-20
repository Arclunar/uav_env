import rospy
from nav_msgs.msg import Odometry
rospy.init_node("T")
pub = rospy.Publisher("/odom_fusion_test",Odometry, queue_size=100)
def convertworld2map(msg):
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
sub = rospy.Subscriber("/mavros/global_position/local", Odometry, convertworld2map)
rospy.spin()
