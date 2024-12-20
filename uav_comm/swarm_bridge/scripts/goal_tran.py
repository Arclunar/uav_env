import rospy
from quadrotor_msgs.msg import GoalSet
from geometry_msgs.msg import PoseStamped

class GoalTransformer:
    def __init__(self):
        rospy.init_node('goal_transformer_node_py')
        self.sub = rospy.Subscriber("/goal_with_id", GoalSet, self.goal_with_id_callback)
        self.pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)

    def goal_with_id_callback(self, msg):
        if(msg.drone_id != 254):
            return
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "world" 
        goal_msg.pose.position.x = float(msg.goal[0])
        goal_msg.pose.position.y = float(msg.goal[1])
        goal_msg.pose.position.z = float(msg.goal[2])
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 0
        goal_msg.pose.orientation.w = 1

        self.pub.publish(goal_msg)

if __name__ == '__main__':
    try:
        goal_transformer = GoalTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass