#include "utm_agent.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utm_agent") ;

  ros::NodeHandle nHandle ;

  agent UTMSystem(nHandle) ;

  ros::spin();
  return 0;
}
