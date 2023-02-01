#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>

int main(int argc,char **argv) {
    ros::init(argc,argv,"grasp_test") ;
    ros::Rate loop_rate(50) ;  

    actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp",true) ; 
    std::cout <<"Waiting for the action server to start" <<std::endl ; 
    sc.waitForServer() ; 

    std::cout <<"Action server started, sending goal" <<std::endl ; 

    franka_gripper::GraspGoal goal ; 
    goal.width=0.05 ; 
    goal.speed=0.1 ; 
    goal.force=2.5 ; 
    goal.epsilon.inner=0.005 ; 
    goal.epsilon.outer=0.005 ; 

    ac.sendGoal(goal) ; 

    bool finished_before_timeout= ac.waitForResult(ros::Duration(30.0)) ; 

    if(finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState() ; 
        std::cout <<"Action finished\n" ; 
    }
    else std::cout <<"Action did not finish before the time out\n" ; 

    ros::Duration(3.0).sleep() ; //sleep for three second

return 0 ; 
}