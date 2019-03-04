#ifndef __GST_TENSOR_ROS_SRC_H__
#define __GST_TENSOR_ROS_SRC_H__

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

G_BEGIN_DECLS

#define GST_TYPE_TENSOR_ROS_SRC \
  (gst_tensor_ros_src_get_type())
#define GST_TENSOR_ROS_SRC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_TENSOR_ROS_SRC,GstTensorRosSrc))
#define GST_TENSOR_ROS_SRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_TENSOR_ROS_SRC,GstTensorRosSrcClass))
#define GST_IS_TENSOR_ROS_SRC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_TENSOR_ROS_SRC))
#define GST_IS_TENSOR_ROS_SRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_TENSOR_ROS_SRC))

typedef struct _GstTensorRosSrc GstTensorRosSrc;
typedef struct _GstTensorRosSrcClass GstTensorRosSrcClass;

struct _GstTensorRosSrc
{
  GstPushSrc parent;

  GstCaps *caps;
  gboolean silent;
  GThread *ros_threaed;   /** ros subscribe thread */

  gchar *topic_name;      /** ROS topic name to subscribe */
  gulong freq_rate;       /** frequency rate to check */
};

struct _GstTensorRosSrcClass 
{
  GstPushSrcClass parent_class;
};

GType gst_ros_src_get_type (void);

G_END_DECLS

#endif /* __GST_TENSOR_ROS_SRC_H__ */