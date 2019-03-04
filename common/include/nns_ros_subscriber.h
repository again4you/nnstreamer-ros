#ifndef __NNS_ROS_SUBSCRIBER_H__
#define __NNS_ROS_SUBSCRIBER_H__

#include <glib.h>
#include <ros/ros.h>

class NnsRosSubscriber {
public:
  NnsRosSubscriber (const gchar *node_name,
    const gchar *topic_name,
    const gulong rate_usec = G_USEC_PER_SEC,
    int argc = 0,
    gchar **argv = NULL);

  virtual int RegisterCallback (ros::NodeHandle *nh, ros::Subscriber *sub) = 0;
  int Start (GThread ** gthread_obj);
  int RequestStop ();

private:
  NnsRosSubscriber () {};

  static gpointer ThreadFunc (gpointer userdata);

  gchar *node_name;
  gchar *topic_name;
  gulong rate_usec;

  ros::NodeHandle *nh;
  ros::Subscriber sub;

  gboolean request_stop;
};
#endif /* __NNS_ROS_SUBSCRIBER_H__ */