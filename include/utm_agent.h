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
#include "Learning/include/NeuralNet.h"

using namespace std ;

typedef unsigned int UINT ;
typedef string PIONEERNAME;
typedef UINT AGENTINDEX;

// Function that takes agent info

class agent
{
public:
ros::Publisher pubAgentGraph ;
vector<ros::Subscriber> subPioneer;


    
    
    matrix2d distances;
    vector<PIONEERNAME> robot_names;
    set<PIONEERNAME> robot_names_called;

    agent_msgs::UtmGraph agentGraph ;
    vector<NeuralNet*> agents;
    matrix1d traffic;


	agent(ros::NodeHandle nh);
	matrix2d loc2distances(std::vector<std::pair<double,double> > agent_locs);
	void trafficCallback(const agent_msgs::AgentMembership& msg);
	void UpdateGraphCosts(UINT index);
	void SwapTraffic(int prevLink, int newLink, PIONEERNAME robot_name);
	void Publish();
	virtual ~agent();


 
};

#endif  // UTM_AGENT
