#include "nn_wrapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utm_system") ;

  ros::NodeHandle nHandle ;

  agent UTMSystem(nHandle) ;

  ros::spin();
  return 0;
}
