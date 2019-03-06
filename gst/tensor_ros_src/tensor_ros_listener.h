#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>

#include "tensor_ros_src.h"

/**
 * @brief Ros Listener class for Int32 type
 */
class Int32RosListener {
  private:
    GstTensorRosSrc *rossrc;
    int payload_size;

  public:
    Int32RosListener (GstTensorRosSrc *rossrc);
    void Callback(const std_msgs::Int32MultiArray msg);
};
