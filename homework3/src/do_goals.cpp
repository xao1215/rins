#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char *argv[]){

    ros::init(argc,argv,"goals_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("GOAL NODE STARTED!");
    MoveBaseClient ac("move_base", true);
    while( !ac.waitForServer(ros::Duration(5.0)) ){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goals[5];
    for(int i = 0; i < 5; i++){
        goals[i].target_pose.header.frame_id = "map";
        goals[i].target_pose.header.stamp = ros::Time::now();
        goals[i].target_pose.pose.orientation.w = 1;
    }
    goals[0].target_pose.pose.position.x = 2.8;
    goals[0].target_pose.pose.position.y = -1.6;
    goals[1].target_pose.pose.position.x = 1.3;
    goals[1].target_pose.pose.position.y = 1.8;
    goals[2].target_pose.pose.position.x = -1.3;
    goals[2].target_pose.pose.position.y = 1.3;
    goals[3].target_pose.pose.position.x = 2.8;
    goals[3].target_pose.pose.position.y = 1.3;
    goals[4].target_pose.pose.position.x = -1.65;
    goals[4].target_pose.pose.position.y = -0.45;

    ac.sendGoal(goals[0]);
    // ac.waitForResult();
    int which = 0;
	ros::Rate rate(2);

    while(ros::ok()) {

        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("status: %s",state.toString().c_str());

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("GOAL N %d REACHED", which+1);
            if(which == 4){break;}
            ac.sendGoal(goals[++which]);
        }else if( state == actionlib::SimpleClientGoalState::REJECTED || state == actionlib::SimpleClientGoalState::RECALLED || state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::LOST ){
            ROS_INFO("GOAL N %d FAILED", which+1);
            if(which == 4){break;}
            ac.sendGoal(goals[++which]);
        }

        rate.sleep();

        // ros::spinOnce();
    }

    return 0;
}