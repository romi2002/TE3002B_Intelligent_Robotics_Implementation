#!/usr/bin/python2.7
import roslib
roslib.load_manifest('puzzlebot_controller')
import rospy
import actionlib
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose
from tf.transformations import euler_from_quaternion

from puzzlebot_controller.msg import MoveAction, MoveActionFeedback

class MoveServer:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("pose", PoseStamped, self._pose_cb)

        self.server = actionlib.ActionServer('puzzlebot_move', MoveAction, self._goal_cb, self._cancel_cb, False)
        self.server.start()

        self.goal = None
        
        self.x_kP = 0.4
        self.x_kD = 0
        self.x_kI = 0.001
        self.last_x_error = 0
        self.x_accum = 0

        self.r_kP = 0.1
        self.r_kD = 0
        self.r_kI = 0
        self.last_yaw_error = 0
        self.r_accum = 0
        self.current_pose = Pose()
        self.start_pose = Pose()

        self.timer = rospy.Timer(rospy.Duration(1.0/100.0), self.update)

    def send_command(self, u, r):
        msg = Twist()
        msg.linear.x = u
        msg.angular.z = r
        #self.cmd_vel_pub.publish(msg)

    def _pose_cb(self, msg):
        self.current_pose = msg.pose

    def _goal_cb(self, g):
        g.set_accepted()
        self.goal = g.get_goal()
        self.start_pose = self.current_pose
        self.last_x_error = 0
        self.x_accum = 0
        self.last_yaw_error = 0
        self.r_accum = 0
    
    def _cancel_cb(self):
        rospy.loginfo("Stopping move!")
        self.send_command(0, 0)
        self.goal = None

    def update(self, event=None):
        if not self.goal:
            return
            self.send_command(0, 0)

        rospy.loginfo(self.goal)
        error_x = (self.current_pose.position.x - self.start_pose.position.x) - self.goal.goal.x
        (_, _, start_yaw) = euler_from_quaternion([
            self.start_pose.orientation.x, 
            self.start_pose.orientation.y,
            self.start_pose.orientation.z,
            self.start_pose.orientation.w])

        (_, _, yaw) = euler_from_quaternion([
            self.current_pose.orientation.x, 
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w])
        error_yaw = (yaw - start_yaw) - self.goal.goal.z
        
        feedback = MoveActionFeedback()
        feedback.feedback.distance_to_target = error_x
        feedback.feedback.angle_to_target = error_yaw
        self.server.publish_feedback(actionlib.GoalStatus.ACTIVE, feedback)
        rospy.loginfo("Error_x: %f, error_yaw: %f" % (error_x, error_yaw))

        if abs(error_x) < 0.05 and abs(error_yaw) < 0.05:
            # Finished with task!
            rospy.loginfo("Finished with goal!")
            self.send_command(0, 0)
            self.goal = None
            self.server.publish_result(None, actionlib.GoalStatus.SUCCEEDED)
            return
        
        x_p = error_x * self.x_kP
        x_d = (error_x - self.last_x_error) * self.x_kD
        self.x_accum += error_x
        x_i = self.x_accum * self.x_kI
        self.last_x_error = error_x

        x_u = x_p + x_d + x_i

        yaw_p = error_yaw * self.r_kP
        yaw_d = (error_yaw - self.last_yaw_error) * self.r_kD
        self.r_accum += error_yaw
        yaw_i = self.r_accum * self.r_kI
        self.last_yaw_error = error_yaw

        yaw_u = yaw_p + yaw_d + yaw_i

        self.send_command(-x_u, yaw_u)

if __name__ == '__main__':
    rospy.init_node('move_action_server')
    server = MoveServer()
    rospy.spin()