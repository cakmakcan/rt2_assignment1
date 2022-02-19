#include <chrono>
#include <cinttypes>
#include <memory>
#include <cstdlib>
#include <inttypes.h>
#include "rt2_assignment1/srv/random_position.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using RandomPosition = rt2_assignment1::srv::RandomPosition;
using Position = rt2_assignment1::srv::Position;
using Command = rt2_assignment1::srv::Command;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{
   class StateMachineClient : public rclcpp::Node
   {	
public:
	bool start = false;
	bool target = false;
	StateMachineClient(const rclcpp::NodeOptions &options) : Node("state_machine", options)
	{
		service_ = this->create_service<Command>("/user_interface", std::bind(&StateMachineClient::user_interface, this, _1,_2,_3));
		client1_ = this->create_client<RandomPosition>("/position_server");
		client2_ = this->create_client<Position>("/go_to_point");
		this->request_1 = std::make_shared<RandomPosition::Request>();
  		this->response_1 = std::make_shared<RandomPosition::Response>();
  		this->request_2 = std::make_shared<Position::Request>();
  		this->response_2 = std::make_shared<Position::Response>();
  		
		timer1_ = this->create_wall_timer(2000ms, std::bind(&StateMachineClient::call_server1, this));
		timer2_ = this->create_wall_timer(2000ms, std::bind(&StateMachineClient::call_server2, this));
    	}
    	
    	void call_server1()
    	{
    		this->request_1->x_max = 5.0;
            	this->request_1->x_min = -5.0;
            	this->request_1->y_max = 5.0;
            	this->request_1->y_min = -5.0;
            	
    		if (start == true){
    				
    			auto response_rp_callback =  [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future){response_1 = future.get();};
			auto future_result = client1_->async_send_request(request_1,response_rp_callback);
    			if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) != rclcpp::FutureReturnCode::SUCCESS)
  			{
    				RCLCPP_ERROR(this->get_logger(), "service call failed :(");
  			}
  			target = true;	
  		}
  	}
  	std::shared_ptr<RandomPosition::Request> request_1;
  	std::shared_ptr<RandomPosition::Response> response_1;
  	
  	void call_server2(){
  		this->request_2->x = this->response_1->x;
            	this->request_2->y = this->response_1->y;
            	this->request_2->theta = this->response_1->theta;
  		
  		if (start == true && target == true){
  			auto response_p_callback =  [this](rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture future){response_2 = future.get();};
			auto future_result = client2_->async_send_request(request_2,response_p_callback);
  			if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) != rclcpp::FutureReturnCode::SUCCESS)
  			{
    				RCLCPP_ERROR(this->get_logger(), "service call failed :(");
  			}
  			
  			std::cout << "\nGoing to the position: x= " << this->request_2->x << " y= " << this->request_2->y << " theta = " << this->request_2->theta << std::endl;
  		
  		}
  		target = false;
  	}
  	
  	std::shared_ptr<Position::Request> request_2;
  	std::shared_ptr<Position::Response> response_2;
  	
private:
	bool user_interface(
        	const std::shared_ptr<rmw_request_id_t> request_header,
        	const std::shared_ptr<Command::Request> req,
        	const std::shared_ptr<Command::Response> res)
        {
            	(void)request_header;

            	if (req->command == "start")
            	{
                	start = true;
            	}
            	else
            	{
                	start = false;
            	}
            	res->ok = true;
            	return true;
            	
        }
        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;
        rclcpp::Client<RandomPosition>::SharedPtr client1_;
        rclcpp::Client<Position>::SharedPtr client2_;
        rclcpp::Service<Command>::SharedPtr service_;
    };
}
// Registering the State Machine Node
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachineClient)  	
  	
  	
      		
