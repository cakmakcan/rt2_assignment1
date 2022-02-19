import rospy
import time
import actionlib
import rt2_assignment1.msg
from rt2_assignment1.srv import Command

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    action_client = actionlib.SimpleActionClient('reaching_goal', rt2_assignment1.msg.PlanningAction)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Please wait, robot stopped!")
            action_client.cancel_all_goals()
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
