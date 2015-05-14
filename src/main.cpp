#include <iostream>

#include <sound_play/sound_play.h>
#include <msgClient.hh>
#include <hatpPlan.hh>


#include <ros/ros.h>
#include "htn_verbalizer/Empty.h"


std::string clientName_ = "hatptester"; //Name should actually be the same as Michelangelo supervisor !!!
msgClient client_;
hatpPlan* plan_;
sound_play::SoundClient* soundClient_;

bool initPlan(htn_verbalizer::Empty::Request &req,
        htn_verbalizer::Empty::Response & res) {

    ROS_INFO("[htn_verbalizer][initPlan] Waiting for a plan");

    std::string answer;

    if (client_.isConnected()) {
        std::pair<std::string, std::string> result = client_.getBlockingMessage();
        //std::cout << "#### Answer : \n" << result.second << std::endl;
        answer = result.second;
    } else {
        ROS_INFO("[htn_verbalizer][WARNING] client not connected!");
        return false;
    }

    ROS_INFO("[htn_verbalizer][initPlan] plan received");

    removeFormatting(answer);
    if (testInputValidity(answer)) {
        plan_ = new hatpPlan(answer);

    } else
        ROS_INFO("[htn_verbalizer][WARNING] unvalid plan received!");

    return true;
}

bool initSpeech(htn_verbalizer::Empty::Request &req,
        htn_verbalizer::Empty::Response & res) {

    if (plan_ != NULL) {
        std::vector<std::string> agents;
        std::string subjectSpeech;
        std::string task;
        std::stringstream ss;

        agents = plan_->getTree()->getRootNode()->getAgents();

        if (agents.size() < 2)
            if (agents[0] == "Robot")
                subjectSpeech = "I ";
            else
                subjectSpeech = "You ";
        else if (std::find(agents.begin(), agents.end(), "Robot") != agents.end())
            subjectSpeech = "We ";
        else
            subjectSpeech = "You ";

        task = plan_->getTree()->getRootNode()->getName();

        for (std::vector<std::string>::iterator it = agents.begin(); it != agents.end(); ++it) {
            if ((*it) != "Robot")
                ss << "Hello " << (*it) << "! ";
        }

        ss << subjectSpeech << "have to " << task;

        soundClient_->say(ss.str());
        sleep(2);
    } else {
        soundClient_->say("Hello, I will compute a plan for us to complete the task!");
        sleep(2);
    }
}

int main(int argc, char ** argv) {


    ros::init(argc, argv, "htn_verbalizer");

    ros::NodeHandle node;

    sound_play::SoundClient soundClient;
    soundClient_ = &soundClient;
    // Init HATP client
    client_.connect(clientName_, "localhost", 5500);


    //Services
    ros::ServiceServer serviceInitPlan = node.advertiseService("htn_verbalizer/init_plan", initPlan);
    ROS_INFO("[Request] Ready to receive a plan.");

    ros::ServiceServer serviceInitSpeech = node.advertiseService("htn_verbalizer/init_speech", initSpeech);
    ROS_INFO("[Request] Ready to init speech.");



    // Set this in a ros service?
    ros::Rate loop_rate(30);


    while (node.ok()) {
        //soundClient_.say("Hello world!");
        sleep(2);

        //if (plan_ != NULL) {
        //    std::cout << "----- Plan : -----" << std::endl;
        //    std::cout << plan_->toString() << std::endl;
        //}

        ros::spinOnce();

        loop_rate.sleep();


    }

}
