#include <iostream>

#include <sound_play/sound_play.h>
#include <msgClient.hh>
#include <hatpPlan.hh>


#include <ros/ros.h>
#include <queue>
#include "htn_verbalizer/Empty.h"
#include "htn_verbalizer/VerbalizeTree.h"


std::string clientName_ = "hatptester"; //Name should actually be the same as Michelangelo supervisor !!!
msgClient client_;
hatpPlan* plan_;
sound_play::SoundClient* soundClient_;
unsigned int nbPartners_ = 0; // This is use so that robot will say "you" if 1 partner and tell name if more than 1

//if(getKnowledge(plan_->getNode(n)->getName(), plan_->getNode(n)->getParameters());

std::string getSubject(std::vector<std::string> agents) {
    if (agents.size() < 2)
        if (agents[0] == "Robot")
            return "I ";
        else
            return "You ";
    else if (std::find(agents.begin(), agents.end(), "Robot") != agents.end())
        return "We ";
    else
        return "You ";
}

double getKnowledge(unsigned int id) {
    //TODO: implement this function!
    hatpNode* node = plan_->getNode(id);
    std::vector<std::string> agents = node->getAgents();

    if (agents.size() == 1)
        if (agents[0] == "Robot")
            return 1.0;

    return 0.0;
}

void tellTask(std::vector<std::string> agents, std::string task) {
    std::stringstream ss;

    ss << getSubject(agents) << "have to " << task;
    soundClient_->say(ss.str());
    printf("[saying] %s", ss.str().c_str());
    sleep(2);
}

std::vector<unsigned int> processNodesOnce(std::vector<unsigned int> currentNodes, unsigned int daddy, bool& stillNeeded) {
    std::vector<unsigned int> processedNodes;

    for (std::vector<unsigned int>::iterator it = currentNodes.begin(); it != currentNodes.end(); ++it) {
        std::vector<unsigned int> children = plan_->getNode((*it))->getSubNodes();

        // If 1 child, verbalize child instead!
        if (children.size() == 1) {
            processedNodes.push_back(children[0]);
            stillNeeded = true;
            continue;

            // If recursive method, verbalize children instead!
        } else if (plan_->getNode((*it))->getName() == plan_->getNode(daddy)->getName()
                && plan_->getNode((*it))->getParameters() == plan_->getNode(daddy)->getParameters()) {
            for (std::vector<unsigned int>::iterator itChildren = children.begin(); itChildren != children.end(); ++itChildren)
                processedNodes.push_back((*itChildren));
            stillNeeded = true;
            continue;
        } else {
            processedNodes.push_back((*it));
        }
    }
    return processedNodes;
}

std::vector<unsigned int> processNodes(std::vector<unsigned int> currentNodes, unsigned int daddy) {
    std::vector<unsigned int> processedNodes;
    bool processStillNeeded = false;

    processedNodes = processNodesOnce(currentNodes, daddy, processStillNeeded);

    while (processStillNeeded) {
        processStillNeeded = false;
        processedNodes = processNodesOnce(processedNodes, daddy, processStillNeeded);
    }
    return processedNodes;
}

void verbalizeNodes(std::vector<unsigned int> currentNodes, unsigned int daddy, double knowledgeThreshold) {
    //TODO: Implement this function!
    if (!currentNodes.empty()) {
        std::queue< std::vector< unsigned int > > queueChildren;
        std::queue< unsigned int > queueDaddies;
        std::vector<unsigned int> processedNodes;

        //pre processing
        processedNodes = processNodes(currentNodes, daddy);


        for (std::vector<unsigned int>::iterator it = processedNodes.begin(); it != processedNodes.end(); ++it) {
            std::stringstream ss;
            printf("verbalizing nodes, current %d\n", (*it));
            // Children
            std::vector<unsigned int> children = plan_->getNode((*it))->getSubNodes();

            if (it == processedNodes.begin()) {
                ss << "To " << plan_->getNode(daddy)->getName() << ", " <<
                        getSubject(plan_->getNode((*it))->getAgents()) << "will first "
                        << plan_->getNode((*it))->getName();

                soundClient_->say(ss.str());
                printf("[saying] %s", ss.str().c_str());
                sleep(4);

                //If not enough knowledge, explain sub nodes
                if (getKnowledge((*it)) < knowledgeThreshold) {
                    queueChildren.push(plan_->getNode((*it))->getSubNodes());
                    queueDaddies.push((*it));
                }
            } else if (it != processedNodes.end()) {
                ss << "Then " << getSubject(plan_->getNode((*it))->getAgents()) << "will "
                        << plan_->getNode((*it))->getName();

                soundClient_->say(ss.str());
                printf("[saying] %s", ss.str().c_str());
                sleep(4);

                //If not enough knowledge, explain sub nodes
                if (getKnowledge((*it)) < knowledgeThreshold) {
                    queueChildren.push(plan_->getNode((*it))->getSubNodes());
                    queueDaddies.push((*it));
                }
            } else {
                ss << "Finally " << getSubject(plan_->getNode((*it))->getAgents()) << "will "
                        << plan_->getNode((*it))->getName();

                soundClient_->say(ss.str());
                printf("[saying] %s", ss.str().c_str());
                sleep(4);

                //If not enough knowledge, explain sub nodes
                if (getKnowledge((*it)) < knowledgeThreshold) {
                    queueChildren.push(plan_->getNode((*it))->getSubNodes());
                    queueDaddies.push((*it));
                }
            }
        }
        while (!queueDaddies.empty()) {
            // TODO: lower the threshold with the depth?
            verbalizeNodes(queueChildren.front(), queueDaddies.front(), knowledgeThreshold);
            queueChildren.pop();
            queueDaddies.pop();
        }
    }
}

bool verbalizeTree(unsigned int n, double knowledgeThreshold) {
    if (plan_ == NULL)
        return false;
    else {
        std::vector<std::string> agents = plan_->getNode(n)->getAgents();
        std::string task = plan_->getNode(n)->getName();

        tellTask(agents, task);

        printf("Task verbalized\n");

        if (getKnowledge(n) < knowledgeThreshold)
            verbalizeNodes(plan_->getNode(n)->getSubNodes(), n, knowledgeThreshold);

        printf("tree verbalized\n");
        return true;
    }
}

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
        std::stringstream ss;

        agents = plan_->getTree()->getRootNode()->getAgents();

        for (std::vector<std::string>::iterator it = agents.begin(); it != agents.end(); ++it) {
            if ((*it) != "Robot")
                ss << "Hello " << (*it) << "! ";
        }

        soundClient_->say(ss.str());
        printf("[saying] %s", ss.str().c_str());
        sleep(2);


        return true;
    } else {
        ROS_INFO("[htn_verbalizer][initSpeech][WARNING] no plan, use init_plan request!");
        //soundClient_->say("Hello, I will compute a plan for us to complete the task!");
        //sleep(2);
        return false;
    }
}

bool verbalizeCurrentPlan(htn_verbalizer::VerbalizeTree::Request &req,
        htn_verbalizer::VerbalizeTree::Response & res) {

    if (plan_ == NULL)
        ROS_INFO("[htn_verbalizer][verbalizeCurrentPlan][WARNING] no plan, use init_plan request!");
    else {
        verbalizeTree(plan_->getTree()->getRootNode()->getID(), req.knowledgeThreshold);
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

    ros::ServiceServer serviceVerbCurrent = node.advertiseService("htn_verbalizer/verbalize_current_plan", verbalizeCurrentPlan);
    ROS_INFO("[Request] Ready to verbalize current plan.");


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
