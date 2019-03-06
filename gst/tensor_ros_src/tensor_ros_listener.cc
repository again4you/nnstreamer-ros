#include <tensor_typedef.h>

#include "tensor_ros_listener.h"

Int32RosListener::Int32RosListener (GstTensorRosSrc *rossrc)
{
  this->rossrc = rossrc;
  this->payload_size = rossrc->count * tensor_element_size[rossrc->datatype];
}

/**
 * @brief callback function for each ROS topic event
 */
void
Int32RosListener::Callback(const std_msgs::Int32MultiArray msg)
{
  gpointer queue_item = g_malloc0 (this->payload_size);

  std::memcpy (queue_item, msg.data.data(), this->payload_size);
  g_async_queue_push (this->rossrc->queue, queue_item);

  GST_DEBUG_OBJECT (this->rossrc, "Queue size: %d\n", g_async_queue_length (this->rossrc->queue));
}
