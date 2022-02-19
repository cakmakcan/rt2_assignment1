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
