#include "ros/ros.h"
#include <std_msgs/String.h>
#include <unistd.h>

ros::Publisher input_pub;
ros::Subscriber goal_sub;
std::string goal = "";

void goalCallback( const std_msgs::String::ConstPtr &msg ) {
    goal = msg->data.c_str();;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "input");
    ros::NodeHandle n;

    input_pub = n.advertise<std_msgs::String>("/input", 10);
    goal_sub = n.subscribe("/goal", 10, &goalCallback);

    while(ros::ok()) {

        std::cout << "spin" << std::flush;

        if(goal == "end"){
            std::cout << "got hint, ending" << std::endl;
            break;
        }

        if(goal != ""){
            std::string input;
            std::cout << "\nDo you know where the robber is?" << std::endl;
            std::getline(std::cin, input);
            ROS_INFO_STREAM( input );
            std_msgs::String msg;
            msg.data = input;
            input_pub.publish(msg);
        }

        sleep(1);
        goal = "";
        ros::spinOnce();
    }
    return 0;

}
