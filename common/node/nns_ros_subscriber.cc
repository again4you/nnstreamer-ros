#include "nns_ros_subscriber.h"

NnsRosSubscriber::NnsRosSubscriber (const gchar *node_name,
    const gchar *topic_name,
    const gulong rate_usec,
    int argc,
    gchar **argv)
{
  this->node_name = g_strdup (node_name);
  this->topic_name = g_strdup (topic_name);
  this->rate_usec = rate_usec;
  this->request_stop = false;

  ros::init (argc, argv, this->node_name);
}

int
NnsRosSubscriber::Start (GThread ** gthread_obj)
{
  this->nh = new ros::NodeHandle();

  /* Call the subclass's method */
  RegisterCallback (this->nh, &this->sub);

  /* Run thread */
  *gthread_obj = g_thread_new ("RosSubThread", (GThreadFunc)NnsRosSubscriber::ThreadFunc, this);

  return 0;
}

int
NnsRosSubscriber::RequestStop ()
{
  this->request_stop = true;
  return 0;
}

gpointer
NnsRosSubscriber::ThreadFunc (gpointer userdata)
{
  NnsRosSubscriber *rossub = static_cast <NnsRosSubscriber *> (userdata);
  while (TRUE) {
    if (rossub->request_stop) {
      g_printerr ("Sub thread is going to stop!\n");
      break;
    }

    /* check the ros topic */
    ros::spinOnce();
    g_usleep (rossub->rate_usec);
  }
  g_thread_exit (GINT_TO_POINTER (0));

  return NULL;
}

