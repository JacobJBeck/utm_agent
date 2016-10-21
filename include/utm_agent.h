#ifndef UTM_AGENT
#define UTM_AGENT

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
    matrix2d loc2distances(std::vector<std::pair<double,double> > agent_locs){
        matrix2d dists = easymath::zeros(agent_locs.size(), agent_locs.size());
        for (int i=0; i<agent_locs.size(); i++){
            for (int j=0; j<agent_locs.size(); j++){
                easymath::XY xyi = easymath::XY(agent_locs[i].first, agent_locs[i].second);
                easymath::XY xyj = easymath::XY(agent_locs[j].first, agent_locs[j].second);

                dists[i][j] = easymath::euclidean_distance(xyi, xyj);
            }
        }
        return dists;
    }

    agent(ros::NodeHandle nh){
        // How many agents do we have?
        int num_agents;
        nh.getParam("num_agents",num_agents);
        std::printf("Getting %i agents...\n", num_agents);
        traffic = matrix1d(num_agents,0);
        
        // Load the braaains...
        string nn_dir;
        nh.getParam("nn_dir", nn_dir);  // directory where all neural nets are stored
        std::printf("Reading from directory %s...\n", nn_dir.c_str());
        for (int i=0; i<num_agents; i++){
            agents.push_back(new NeuralNet());
            agents.back()->load(nn_dir+"/net_"+to_string(i)+".csv");
        }

        // Robot info
        int num_robots;
        nh.getParam("num_robots",num_robots);
        std::printf("Getting names of %i robots...\n", num_robots);

        robot_names = std::vector<std::string>();
        for (int i=0; i<num_robots; i++){
            robot_names.push_back("pioneer"+std::to_string(i));
        }

        for (PIONEERNAME n : robot_names){
            string topic = n +"/membership";
            ros::Subscriber sub = nh.subscribe(topic, 50, &agent::trafficCallback, this);
            subPioneer.push_back(sub);
        }

        /*
         * std::string agent_locations_file;
        nh.getParam("agent_locations_file", agent_locations_file);
        // read the csv file
        std::vector<std::pair<double,double> > agent_locations = easyio::read_pairs<std::pair<double,double> >(agent_locations_file);
        distances = loc2distances(agent_locations);
        std::printf("Got %i distances...\n", static_cast<int>(distances.size()));
        */

        pubAgentGraph = nh.advertise<agent_msgs::UtmGraph>("/utm_graph", 10, true) ; // set boolean to false if no need to latch
        
        // Finished initialization
        ROS_INFO("Agents ready!");
    }

    ~agent() {}

private:
    vector<ros::Subscriber> subPioneer;
    ros::Publisher pubAgentGraph ;
    matrix2d distances;
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

    void SwapTraffic(int prevLink, int newLink, PIONEERNAME robot_name)
    {
        if(prevLink > 0 && newLink > 0)
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
        }else if(prevLink > 0){
            traffic[prevLink]--;
            UpdateGraphCosts(prevLink);
        }
    }

    void Publish(){
        // Publish new cost graph
        pubAgentGraph.publish(agentGraph) ;
    }
};

#endif  // UTM_AGENT
