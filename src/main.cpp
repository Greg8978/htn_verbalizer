#include <iostream>

#include <sound_play/sound_play.h>
#include <msgClient.hh>
#include <hatpPlan.hh>


#include <ros/ros.h>
#include <queue>
#include "htn_verbalizer/Empty.h"
#include "htn_verbalizer/VerbalizeTree.h"

#include "toaster_msgs/GetFactValue.h"
#include "toaster_msgs/Fact.h"


std::string clientName_ = "hatptester"; //Name should actually be the same as Michelangelo supervisor !!!
msgClient hatpClient_;
hatpPlan* plan_;
sound_play::SoundClient* soundClient_;
ros::ServiceClient* knowledgeClient_;
unsigned int nbPartners_ = 0; // This is use so that robot will say "you" if 1 partner and tell name if more than 1

/*std::map<std::string, std::string> PlanToSpeech_;


std::string initPlanNamesToSpeech() {

    PlanToSpeech_["Blue_Cube"] = "blue cube ";
    PlanToSpeech_["Red_Cube"] = "red cube ";
    PlanToSpeech_["Green_Cube"] = "green cube ";
}
 */

std::string getParameterClass(std::string object) {
    if ("Green_Cube" || "Blue_Cube" || "Red_Cube")
        return "Cubes";
    else
        return object;
}

std::string planToKnowledge(std::string name) {
    if (name == "Human_1")
        return "HERAKLES_HUMAN1";
    else if (name == "Human_2")
        return "HERAKLES_HUMAN2";
    else if (name == "Robot")
        return "pr2";
    else if (name == "Blue_Cube")
        return "BlueCube ";
    else if (name == "Red_Cube")
        return "RedCube";
    else if (name == "Green_Cube")
        return "GreenCube";
    else
        return name;
}

std::string planToKnowledge(unsigned int id) {
    std::stringstream ss;

    // Some knowledge apply to class of objects, other on object itself
    if (plan_->getNode(id)->getName() == "Handle")
        ss << plan_->getNode(id)->getName() << "_" << planToKnowledge(plan_->getNode(id)->getParameters()[1]);
    else if (plan_->getNode(id)->getName() == "PlaceOnStack")
        ss << plan_->getNode(id)->getName() << "_" << getParameterClass(plan_->getNode(id)->getParameters()[1]);
    else if (plan_->getNode(id)->getName() == "Pick")
        ss << plan_->getNode(id)->getName() << "_" << getParameterClass(plan_->getNode(id)->getParameters()[1]);
    else if (plan_->getNode(id)->getName() == "HandleOperation")
        ss << planToKnowledge(plan_->getNode(id)->getParameters()[2]) << "_" << getParameterClass(plan_->getNode(id)->getParameters()[1]);
    else
        ss << plan_->getNode(id)->getName();
    return ss.str();
}

std::string planNamesToSpeech(std::string plan) {

    if (plan == "Blue_Cube")
        return "blue cube ";
    else if (plan == "Red_Cube")
        return "red cube ";
    else if (plan == "Green_Cube")
        return "green cube ";
    else if (plan == "Stickers")
        return "put stickers on ";
    else
        return plan;
}

std::string nodeToText(unsigned int id) {
    std::stringstream ss;

    if (plan_->getNode(id)->getName() == "Handle")
        ss << "handl the " << planNamesToSpeech(plan_->getNode(id)->getParameters()[1]);
    else if (plan_->getNode(id)->getName() == "PlaceOnStack")
        ss << "place the " << planNamesToSpeech(plan_->getNode(id)->getParameters()[1]) << "on the stack ";
    else if (plan_->getNode(id)->getName() == "Pick")
        ss << "pick the " << planNamesToSpeech(plan_->getNode(id)->getParameters()[1]);
    else if (plan_->getNode(id)->getName() == "HandleOperation")
        ss << planNamesToSpeech(plan_->getNode(id)->getParameters()[2]) << " the " << planNamesToSpeech(plan_->getNode(id)->getParameters()[1]);
    else
        ss << planNamesToSpeech(plan_->getNode(id)->getName());
    return ss.str();
}

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

// We decide here:
// -> to return 1.0 if it concerns only Robot
// -> to return the lower knowledge value if it concerns several agents

double getKnowledge(unsigned int id) {
    //TODO: implement this function!
    hatpNode* node = plan_->getNode(id);
    std::vector<std::string> agents = node->getAgents();
    std::string agent;

    double knowledge = 1.0;
    double curKnowledge = 0.0;

    if (agents.size() == 1) {
        if (agents[0] == "Robot") {
            return 1.0;
        }
    } else {
        std::vector<std::string>::iterator it = std::find(agents.begin(), agents.end(), "Robot");
        if (it != agents.end()) {
            agents.erase(it);
        }
    }

    for (std::vector<std::string>::iterator it = agents.begin(); it != agents.end(); ++it) {

        toaster_msgs::GetFactValue getKnowledge;
        getKnowledge.request.agentName = planToKnowledge("Robot");
        getKnowledge.request.reqFact.property = planToKnowledge(id);
        getKnowledge.request.reqFact.subjectName = planToKnowledge((*it));

        ROS_INFO("[Request] we request knowledge in Robot model: %s %s \n", planToKnowledge((*it)).c_str(), planToKnowledge(id).c_str());

        if (knowledgeClient_->call(getKnowledge)) {
            // If this agent has less knowledge, we keep his level
            curKnowledge = getKnowledge.response.resFact.doubleValue;

            if (curKnowledge == 0.0)
                return 0.0;
            else
                knowledge = curKnowledge;
        }
    }
    return knowledge;
}

void tellTask(std::vector<std::string> agents, std::string task) {
    std::stringstream ss;

    ss << getSubject(agents) << "have to " << task;
    soundClient_->say(ss.str());
    sleep(3);
    printf("[saying] %s\n", ss.str().c_str());
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
            //printf("verbalizing nodes, current %d\n", (*it));
            // Children
            std::vector<unsigned int> children = plan_->getNode((*it))->getSubNodes();

            if (it == processedNodes.begin()) {
                ss << "To " << nodeToText(daddy) << ", " <<
                        getSubject(plan_->getNode((*it))->getAgents()) << "will first "
                        << nodeToText((*it));

                soundClient_->say(ss.str());
                printf("[saying] %s\n", ss.str().c_str());
                sleep(4);

                //If not enough knowledge, explain sub nodes
                if (getKnowledge((*it)) < knowledgeThreshold) {
                    queueChildren.push(plan_->getNode((*it))->getSubNodes());
                    queueDaddies.push((*it));
                }
            } else if (((*it) != processedNodes.back()) || ((*it) == processedNodes[1])) {
                ss << "Then " << getSubject(plan_->getNode((*it))->getAgents()) << "will "
                        << nodeToText((*it));

                soundClient_->say(ss.str());
                printf("[saying] %s\n", ss.str().c_str());
                sleep(3);

                //If not enough knowledge, explain sub nodes
                if (getKnowledge((*it)) < knowledgeThreshold) {
                    queueChildren.push(plan_->getNode((*it))->getSubNodes());
                    queueDaddies.push((*it));
                }
            } else {
                ss << "Finally " << getSubject(plan_->getNode((*it))->getAgents()) << "will "
                        << nodeToText((*it));

                soundClient_->say(ss.str());
                printf("[saying] %s\n", ss.str().c_str());
                sleep(3);

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

    if (hatpClient_.isConnected()) {
        std::pair<std::string, std::string> result = hatpClient_.getBlockingMessage();
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
        printf("[saying] %s\n", ss.str().c_str());
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
    hatpClient_.connect(clientName_, "localhost", 5500);


    ros::ServiceClient knowledgeClient = node.serviceClient<toaster_msgs::GetFactValue>("/belief_manager/get_fact_value", true);
    knowledgeClient_ = &knowledgeClient;


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
        //sleep(2);

        //if (plan_ != NULL) {
        //    std::cout << "----- Plan : -----" << std::endl;
        //    std::cout << plan_->toString() << std::endl;
        //}

        ros::spinOnce();

        loop_rate.sleep();


    }

}
