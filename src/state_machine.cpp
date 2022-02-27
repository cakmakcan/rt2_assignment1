/** /package rt2_assignment1
* 
* /file state_machine.cpp
* /brief Node describing the state machine implementation 
* /author Can CAKMAK
* /version 0.1
* /date 20.02.2022
*
*
*
* /details 
*
* Subscribes to: <BR>
*    None
* 
* Publishes to: <BR>
*    None
* 
* Client: <BR>
*    /position_server
* 	
* 
* Actiom Client: <BR>
*    /go_to_point
*
* Services: <BR>
*  /user_interface
*
* Description: <BR>
*
* By the help of an user interface, the user is able to make the robot start 
* by entering the 1 integer value, the robot starts moving. There is one boolean 
* value which becomes true and then call the  position_service
* which retrieves the random goal position to reach from the RandomPosition.srv custom 
* service, sends the random position as the action server goal, waits for the robot 
* to reach the designated position
*
*/

#include <ros/ros.h>
#include <rt2_assignment1/Command.h>
#include <rt2_assignment1/Position.h>
#include <rt2_assignment1/RandomPosition.h>
#include <rt2_assignment1/PlanningAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <unistd.h>

/**
 * /brief This function is the callback function of the service for server.
 * /param req  the request received from the client of the user_interface.py. 
 * /param res  the response has not been used 
 * 
 * /retval A boolean value
 * 
 * This function allows to initialize the global variable *"start"* to true
 * wether the command received consists in a "start" string. Otherwise, it is
 * initialized as *"false"*
 * 
 */
 
bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
 * /brief The main function
 * 
 * /retval Always returns zero
 * 
 * This function implements:
 * -# the state_machine node
 * -# the service
 * -# the client
 * -# the action client
 * -# the two custom messages RandomPosition and GoalReachingGoal
 * If start var is set to "true", then we call the RandomPosition service
 * and we wait for the action server to start. Once started, the goal fields
 * are populated with the retrieved random values. Then, the goal is sent 
 * to the action server. Then a check over the robot achievement within a specifed 
 * time interval has been configured.
 * 
 *
 * 
 */

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   //ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   actionlib::SimpleActionClient<rt2_assignment1::PlanningAction> ac("reaching_goal", true);
   ros::Publisher pub_stat =n.advertise<std_msgs::String>("action_status", 100);
   std_msgs::String status;
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::Position p;
   
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		int i;
   		rt2_assignment1::PlanningGoal goal;
   		client_rp.call(rp);
   		//p.request.x = rp.response.x;
   		//p.request.y = rp.response.y;
   		//p.request.theta = rp.response.theta;
   	
  		goal.x = rp.response.x;
  		goal.y = rp.response.y;
  		goal.theta = rp.response.theta;
  		ac.sendGoal(goal);
  		ROS_INFO("Sending goal");
  		status.data=ac.getState().toString().c_str();
  		pub_stat.publish(status);
  		
  		
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << " theta = " <<goal.theta << std::endl;
   		
		//wait for the action to return		
		bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
		
		if (finished_before_timeout){
  			ROS_INFO("Action finished: %s",ac.getState().toString().c_str());
			//status SUCCEEDED or PREEMPTED
			status.data = ac.getState().toString().c_str();
			pub_stat.publish(status);
  		}
  		
   		else{
   			ROS_INFO("Action did not finish before the time out.");
    			//status ACTIVE
			status.data = ac.getState().toString().c_str();
			pub_stat.publish(status);
   		}
   	}
   	
   	ros::Duration(0.5).sleep();
   }
   	  		 		
   
   return 0;
}
