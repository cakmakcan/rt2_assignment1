#! /usr/bin/env python


import rospy
import roslib
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
from rt2_assignment1.srv import Command
import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg

#from rt2_assignment1.msg import PlanningAction, PlanningGoal

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None
pose_ = Pose()

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

def clbk_odom(msg):
    global position_
    global yaw_
    global pose_

    # position
    position_ = msg.pose.pose.position
    pose_=msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    
class PlanningAction():
	feedback_ = rt2_assignment1.msg.PlanningActionFeedback()
	result_ = rt2_assignment1.msg.PlanningActionResult()
	def __init__(self):
		self.server = actionlib.SimpleActionServer('reaching_goal', rt2_assignment1.msg.PlanningAction, self.go_to_point, auto_start = False)
		self.server.start()
		
	def go_to_point(self,goal):
		
		desired_position_ = Point()
		desired_position_.x = goal.x
		desired_position_.y = goal.y
		des_yaw = goal.theta
		change_state(0)
		success = True
		rate = rospy.Rate(20)
		
		while not rospy.is_shutdown():
			if self.server.is_preempt_requested():
				rospy.loginfo('Goal was preempted')
				self.server.set_preempted()
				success = False
				done()
				break
			
			if success:
				if state_ == 0:
					fix_yaw(desired_position_)
				elif state_ == 1:
					go_straight_ahead(desired_position_)
				elif state_ == 2:
					fix_final_yaw(des_yaw)
				elif state_ == 3:
					done()
					rospy.loginfo('Goal: Succeeded!')
					self.result_.result = True
					self.server.set_succeeded(self.result_)
					break
				else:
					rospy.logerr('Unknown state!') 
			rate.sleep()

def main():
    global pub_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    #service = rospy.Service('/go_to_point', Position, go_to_point)
    action_server = PlanningAction()
    rospy.spin()

if __name__ == '__main__':
    main()
