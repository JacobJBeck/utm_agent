#ifndef NN_WRAPPER
#define NN_WRAPPER

#include <ros/ros.h>
#include "std_msgs/UInt8.h"
#include "agent_msgs/AgentMembership.h" // custom message for reporting parent (current) and child (next) agents
#include "agent_msgs/UtmGraph.h" // custom message for reporting link traversal costs over entire graph
#include <vector>
#include <map>
#include <functional>
#include <boost/function.hpp>

#include "NeuralNet.h"

using namespace std ;

typedef unsigned int UINT ;
typedef string PIONEERNAME;
typedef UINT AGENTINDEX;

// Function that takes agent info

class agent
{
public:
    agent(ros::NodeHandle nh){
        ros::param::get("utm_agent/robot_names",robot_names);
        for (PIONEERNAME n : robot_names){
            string topic = n +"/membership";
            ros::Subscriber sub = nh.subscribe(topic, 50, &agent::trafficCallback, this);
            subPioneer.push_back(sub);
        }
        ros::param::get("utm_agent/distances", distances);
        int n_agents;
        ros::param::get("utm_agent/n_agents",n_agents);
        string nn_dir;
        ros::param::get("utm_agent/nn_dir", nn_dir);  // directory where all neural nets are stored
        for (int i=0; i<n_agents; i++){
            agents.push_back(new NeuralNet());
            agents.back()->load(nn_dir+"/net_"+to_string(i)+".txt");
        }

        pubAgentGraph = nh.advertise<agent_msgs::UtmGraph>("utm_graph", 10, true) ; // set boolean to false if no need to latch
    }

    ~agent() {}

private:
    vector<ros::Subscriber> subPioneer;
    ros::Publisher pubAgentGraph ;
    matrix1d distances;
    vector<PIONEERNAME> robot_names;
    set<PIONEERNAME> robot_names_called;

    agent_msgs::UtmGraph agentGraph ;
    vector<NeuralNet*> agents;
    matrix1d traffic;

    void trafficCallback(const agent_msgs::AgentMembership& msg){//, const std::string &topic){
        SwapTraffic(msg.parent, msg.child, msg.robot_name);
        Publish();
    }

    // input refers to agent ID
    void UpdateGraphCosts(UINT index){
        double cost = agents.at(index)->operator()(traffic)[0] ;
        agentGraph.policy_output_costs[index] = cost ;
    }

    void SwapTraffic(UINT prevLink, UINT newLink, PIONEERNAME robot_name)
    {
        bool first_call = robot_names_called.count(robot_name)==0;
        if (first_call){
            robot_names_called.insert(robot_name);
        } else {
            traffic[prevLink]--;
            UpdateGraphCosts(prevLink);
        }
        traffic[newLink]++;
        UpdateGraphCosts(newLink);
    }

    void Publish(){
        // Publish new cost graph
        pubAgentGraph.publish(agentGraph) ;
    }
};

#endif  // NN_WRAPPER
