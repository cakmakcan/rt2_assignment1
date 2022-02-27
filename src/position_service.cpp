/**
 * \file position_service.cpp 
 * \brief This file generates random two numbers for a robot to reach a random target
 * \author Can CAKMAK
 * \version 0.1
 * \date 20.02.2022
 *
 * \details
 * Services : <BR>
 *    * \position server 
 *
 * Description :
 *
 * This node advertises a position service. When the service a required, a request
 * containing minimum a maximum values for the x and y position.
*
 */
  
#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"



/**
 * \brief random number generator 
 * \param M defines minimum number
 * \param N defines maximum number
 
 * \return the random number
 *
 * This function generates a random number between M and N
 */
 
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
 * \brief callback function 
 * \param &req is used to have the request from client
 * \param &res is used to send a response to client
 
 * \return always true
 *
 * This function is a callback function exploited by main in order to advertise the service
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 * \brief main function 
 * \param argc is number of arguments
 * \param argv is array of arguments
 
 * \return always 0
 *
 * This function initialize position service node and service server
 */
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
