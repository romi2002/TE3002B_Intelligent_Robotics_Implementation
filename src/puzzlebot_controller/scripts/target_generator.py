import rospy
from geometry_msgs.msg import Pose2D
import csv
import math
        
# Reads target pose from csv
def read_csv(filename):
    point_list = []
    with open(filename, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            print(row)
            point_list.append([float(n) for n in row])
    return point_list

def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping Target Pose Generator")

class TargetPoseGenerator():
    def __init__(self):
        self.targets = read_csv("points.csv")

        self.current_target_index = 0
        self.epsilon = 0.1
        # Create Publishers and Subscribers
        rospy.Subscriber('current_pose',Pose2D,self.robot_pose_cb)
        self.target_pose_pub = rospy.Publisher('target_pose', Pose2D, queue_size=10)

    def robot_pose_cb(self, msg: Pose2D):
        tx, ty = self.targets[self.current_target_index]
        x = msg.x
        y = msg.y
        dist = math.hypot(x - tx, y - ty)

        rospy.loginfo(f"Tx: {tx} Ty: {ty} dist: {dist}")

        if dist < self.epsilon:
            # Move to next point
            self.current_target_index += 1

        if self.current_target_index >= len(self.targets):
            # Reached end of pose!
            self.current_target_index = 0
            rospy.loginfo("Looping back to start of traj :)")
            #rospy.signal_shutdown("Reached end of traj")
            
        self.send_target_pose()

    def send_target_pose(self):
        tx, ty = self.targets[self.current_target_index]

        target_msg = Pose2D()
        target_msg.x = float(tx)
        target_msg.y = float(ty)

        self.target_pose_pub.publish(target_msg)
        
if __name__ == '__main__':
    rospy.init_node('target_pose_generator')
    rate = rospy.Rate(25)
    rospy.on_shutdown(stop)

    generator = TargetPoseGenerator()
    rospy.loginfo("Pose Generator is Running")

    while not rospy.is_shutdown():
        generator.send_target_pose()
        rate.sleep()