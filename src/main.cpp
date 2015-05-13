#include <iostream>

#include <sound_play/sound_play.h>
#include <msgClient.hh>
#include <hatpPlan.hh>


#include <ros/ros.h>
#include "htn_verbalizer/Empty.h"


std::string clientName_ = "hatptester"; //Name should actually be the same as Michelangelo supervisor !!!
msgClient client_;
hatpPlan* plan_;

bool initPlan(htn_verbalizer::Empty::Request &req,
        htn_verbalizer::Empty::Response & res) {

    ROS_INFO("[htn_verbalizer][initPlan] Waiting for a plan");

    std::string answer;

    if (client_.isConnected()) {
        std::pair<std::string, std::string> result = client_.getBlockingMessage();
        std::cout << "#### Answer : \n" << result.second << std::endl;
        answer = result.second;
    } else {
        ROS_INFO("[htn_verbalizer][WARNING] client not connected!");
        return false;
    }

    ROS_INFO("[htn_verbalizer][initPlan] plan received");

    removeFormatting(answer);
    if (testInputValidity(answer)) {
        plan_ = new hatpPlan(answer);

        std::cout << "----- Plan : -----" << std::endl;
        std::cout << plan_->toString() << std::endl;
    } else
        ROS_INFO("[htn_verbalizer][WARNING] unvalid plan received!");


    return true;
}

int main(int argc, char ** argv) {


    ros::init(argc, argv, "htn_verbalizer");

    ros::NodeHandle node;
    sound_play::SoundClient sc;


    // Init HATP client
    client_.connect(clientName_, "localhost", 5500);


    //Services
    ros::ServiceServer serviceInitPlan = node.advertiseService("htn_verbalizer/init_plan", initPlan);
    ROS_INFO("[Request] Ready to receive a plan.");


    // Set this in a ros service?
    ros::Rate loop_rate(30);


    while (node.ok()) {
        sc.say("Hello world!");
        sleep(2);

        loop_rate.sleep();


    }

}
