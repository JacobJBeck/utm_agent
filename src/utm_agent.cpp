#include "utm_agent.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "utm_agent") ;

    ros::NodeHandle nHandle ;

    agent UTMSystem(nHandle) ;
    ros::spin();
    return 0;
}




matrix2d agent::loc2distances(std::vector<std::pair<double,double> > agent_locs){
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

agent::agent(ros::NodeHandle nh){
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
void agent::trafficCallback(const agent_msgs::AgentMembership& msg){//, const std::string &topic){
    SwapTraffic(msg.parent, msg.child, msg.robot_name);
    Publish();
}

// input refers to agent ID
void agent::UpdateGraphCosts(UINT index){
    matrix1d ti= matrix1d(1,traffic[index]);

    auto ai = agents.at(index);
    matrix1d action = ai->operator()(ti);

    double cost = action.at(0);

    if (agentGraph.policy_output_costs.size()==0){
        agentGraph.policy_output_costs = matrix1d(traffic.size(),0.0);
    }
    agentGraph.policy_output_costs[index] = cost;
}

void agent::SwapTraffic(int prevLink, int newLink, PIONEERNAME robot_name)
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

void agent::Publish(){
    // Publish new cost graph
    pubAgentGraph.publish(agentGraph) ;
}

agent::~agent() {}
