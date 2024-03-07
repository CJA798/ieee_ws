#ifndef _ROS_H_
#define _ROS_H_

#define MAX_PUBLISHERS 10
#define MAX_SUBSCRIBERS 5
#define SUB_BUFFER_SIZE 512
#define PUB_BUFFER_SIZE 512

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
  // default is 25, 25, 256, 512  
  typedef NodeHandle_<ArduinoHardware, MAX_PUBLISHERS, MAX_SUBSCRIBERS, SUB_BUFFER_SIZE, PUB_BUFFER_SIZE> NodeHandle;

  // This is legal too and will use the default 25, 25, 512, 512
  //typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#endif
