/*
 * GStreamer
 * Copyright (C) 2005 Thomas Vander Stichele <thomas@apestaart.org>
 * Copyright (C) 2005 Ronald S. Bultje <rbultje@ronald.bitfreak.net>
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <tensor_filter_custom.h>
#include <tensor_typedef.h>
#include <string.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>

#include "tensor_ros_src.h"

GST_DEBUG_CATEGORY_STATIC (gst_tensor_ros_src_debug);
#define GST_CAT_DEFAULT gst_tensor_ros_src_debug

#define SUPPORTED_CAPS_STRING "application/octet-stream"

/**
 * @brief Ros Listener class for Int32 type
 */
class Int32RosListener {
  private:
    GstTensorRosSrc *rossrc;
    int payload_size;

  public:
    Int32RosListener (GstTensorRosSrc *rossrc, int count);
    int GetPayloadSize ();
    void Callback(const std_msgs::Int32MultiArray msg);
};

Int32RosListener::Int32RosListener (GstTensorRosSrc *rossrc, int count)
{
  this->rossrc = rossrc;
  this->payload_size = count * tensor_element_size[rossrc->datatype];
}

int
Int32RosListener::GetPayloadSize ()
{
  return this->payload_size;
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

/** ROS Listener instance for each type */
static Int32RosListener *int32RosListener;

/**
 * @brief Concrete Ros Subscriber class
 */
class TensorRosSub : public NnsRosSubscriber {
  private:
    GstTensorRosSrc *rossrc;

  public:
    TensorRosSub (const gchar *node_name,
      const gchar *topic_name,
      GstTensorRosSrc *rossrc,
      const gulong rate_usec = G_USEC_PER_SEC,
      int argc = 0,
      gchar **argv = NULL) : NnsRosSubscriber (node_name, topic_name, rate_usec, argc, argv)
      {
        this->rossrc = rossrc;
      }

      virtual int RegisterCallback (ros::NodeHandle *nh, ros::Subscriber *sub);
};

/**
 * @brief register callback function to subscribe ROS topic
 */
int
TensorRosSub::RegisterCallback (ros::NodeHandle *nh, ros::Subscriber *sub)
{
  GST_DEBUG_OBJECT (this->rossrc, "[%s]", __func__);

  /* TODO: add more type */
  *sub = nh->subscribe (this->topic_name, 1000, &Int32RosListener::Callback, int32RosListener);

  return 0;
}

/**
 * @brief tensor_ros_src properties
 */
enum
{
  PROP_0,
  PROP_SILENT,      /*<< Slient mode for debug */
  PROP_TOPIC,       /*<< ROS topic name to subscribe */
  PROP_FREQ_RATE,   /*<< frequency rate to check the published topic */
  PROP_DATATYPE,    /*<< Primitive datatype of ROS topic */
};

/**
 * @brief Template for ros source pad
 */
static GstStaticPadTemplate src_pad_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (SUPPORTED_CAPS_STRING));

#define gst_tensor_ros_src_parent_class parent_class
G_DEFINE_TYPE (GstTensorRosSrc, gst_tensor_ros_src, GST_TYPE_PUSH_SRC);

/** GObject method implementation */
static void gst_tensor_ros_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_tensor_ros_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);
static void gst_tensor_ros_src_dispose (GObject * object);

/** GstElement method implementation */
static GstStateChangeReturn
gst_tensor_ros_src_change_state (GstElement * element, GstStateChange transition);

/** GstPushSrc method implementation */
static GstFlowReturn 
gst_tensor_ros_src_create (GstPushSrc * src, GstBuffer ** buffer);

/** Utility function */
static const gchar* str_type[] = {"int32", "uint32",
  "int16", "uint16",
  "int8", "uint8",
  "float64", "float32",
  "int64", "uint64"};

static tensor_type
get_tensor_type (const gchar * type_name)
{
  int type_count = sizeof(str_type);
  for (int i = 0; i < type_count; ++i) {
    if (!g_strcmp0 (type_name, str_type[i]))
      return static_cast<tensor_type> (i);
  }
  return _NNS_END;
}

static const gchar *
get_string_type (tensor_type type)
{
  return str_type[type];
}

/**
 * @brief initialize the rossrc's class
 */
static void
gst_tensor_ros_src_class_init (GstTensorRosSrcClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS (klass);
  GstPushSrcClass *gstpushsrc_class = GST_PUSH_SRC_CLASS (klass);

  /* GObject method */
  gobject_class->set_property = gst_tensor_ros_src_set_property;
  gobject_class->get_property = gst_tensor_ros_src_get_property;
  gobject_class->dispose = gst_tensor_ros_src_dispose;  

  /* GstElement method for state change */
  gstelement_class->change_state =
    GST_DEBUG_FUNCPTR (gst_tensor_ros_src_change_state);

  /* GstPushSrc method */
  gstpushsrc_class->create = gst_tensor_ros_src_create;

  /* Add property */
  g_object_class_install_property (gobject_class, PROP_SILENT,
    g_param_spec_boolean ("silent", "Silent", "Produce verbose output ?",
        FALSE, G_PARAM_READWRITE));
  
  g_object_class_install_property (gobject_class, PROP_TOPIC,
    g_param_spec_string ("topic", "Topic",
      "ROS Topic Name for subscription", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_FREQ_RATE,
    g_param_spec_ulong ("freqrate", "Frequency_Rate",
      "Frequency rate for checking Ros Topic(Hz, Default: 1Hz)",
      1, G_USEC_PER_SEC, 1, G_PARAM_READWRITE));

  g_object_class_install_property (gobject_class, PROP_DATATYPE,
    g_param_spec_string ("datatype", "Datatype",
      "Primitive datatype of target ROS topic", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  gst_element_class_add_static_pad_template (gstelement_class,
    &src_pad_template);

  gst_element_class_set_static_metadata (gstelement_class,
    "TensorRosSrc",
    "Source/Tensor",
    "Source element to get the topic data from ROS node",
    "Samsung Electronics Co., Ltd.");
}

/**
 * @brief Initialize ensor_ros_src element
 */
static void
gst_tensor_ros_src_init (GstTensorRosSrc * rossrc)
{
  /* set the default properties */
  rossrc->silent = FALSE;
  rossrc->topic_name = NULL;
  rossrc->freq_rate = G_USEC_PER_SEC;
  rossrc->queue = g_async_queue_new ();
}

/**
 * @brief Dispose allocated resources
 */
static void
gst_tensor_ros_src_dispose (GObject * object)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (object);

  if (rossrc->caps)
    gst_caps_unref (rossrc->caps);

  if (rossrc->queue)
    g_async_queue_unref (rossrc->queue);

  if (rossrc->topic_name)
    g_free (rossrc->topic_name);

  G_OBJECT_CLASS (parent_class)->dispose (object);
}

/**
 * @brief Setter for tensor_ros_src properties
 */
static void
gst_tensor_ros_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (object);

  switch (prop_id) {
    case PROP_SILENT:
      rossrc->silent = g_value_get_boolean (value);
      break;

    case PROP_TOPIC:
      rossrc->topic_name = g_strdup(g_value_get_string (value));
      GST_DEBUG_OBJECT (rossrc, "topic name: %s\n", rossrc->topic_name);
      break;

    case PROP_FREQ_RATE:
      rossrc->freq_rate = G_USEC_PER_SEC / g_value_get_ulong (value);
      GST_DEBUG_OBJECT (rossrc, "Freq Hz: %lu\n", G_USEC_PER_SEC / rossrc->freq_rate);
      break;

    case PROP_DATATYPE:
      rossrc->datatype = get_tensor_type (g_value_get_string (value));
      GST_DEBUG_OBJECT (rossrc, "Datatype: %s\n", get_string_type(rossrc->datatype));
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/**
 * @brief Getter for tensor_ros_src properties
 */
static void
gst_tensor_ros_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (object);

  switch (prop_id) {
    case PROP_SILENT:
      g_value_set_boolean (value, rossrc->silent);
      break;
    
    case PROP_TOPIC:
      g_value_set_string (value, rossrc->topic_name);
      break;

    case PROP_FREQ_RATE:
      g_value_set_uint (value, G_USEC_PER_SEC / rossrc->freq_rate);
      break;

    case PROP_DATATYPE:
      g_value_set_string (value, get_string_type(rossrc->datatype));
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/**
 * @brief Getter for tensor_ros_src properties
 */
static GstStateChangeReturn
gst_tensor_ros_src_change_state (GstElement * element, GstStateChange transition)
{
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (element);
  GstStateChangeReturn ret;

  /* handle before change */
  switch (transition) {
    case GST_STATE_CHANGE_NULL_TO_READY:
      /* create thread */
      GST_DEBUG_OBJECT (rossrc, "State is changed: GST_STATE_CHANGE_NULL_TO_READY\n");

      /* TODO: check type & it size from parameter */
      int32RosListener = new Int32RosListener(rossrc, 10);
      rossrc->ros_sub = new TensorRosSub ("TensorRosSub", rossrc->topic_name,
        rossrc, rossrc->freq_rate);
      rossrc->ros_sub->Start (&rossrc->thread);
      break;

    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (parent_class)->change_state (element, transition);

  /* handle after change */
  switch (transition) {
    case GST_STATE_CHANGE_READY_TO_NULL:
      /* stop thread */
      GST_DEBUG_OBJECT (rossrc, "State is changed: GST_STATE_CHANGE_READY_TO_NULL\n");
      rossrc->ros_sub->RequestStop ();
      break;

    default:
      break;
  }

  return ret;
}

/**
 * @brief Push GstBuffer which contains the subscribed ROS data
 */
static GstFlowReturn
gst_tensor_ros_src_create (GstPushSrc * src, GstBuffer ** buffer)
{
  gpointer queue_item = NULL;
  GstTensorRosSrc *rossrc = GST_TENSOR_ROS_SRC (src);
  GstBuffer *buf = NULL;
  GstMemory *mem;
  GstMapInfo info;
  gsize size = int32RosListener->GetPayloadSize();

  /* get item from queue */
  queue_item = g_async_queue_try_pop (rossrc->queue);

  buf = gst_buffer_new ();

  mem = gst_allocator_alloc (NULL, size, NULL);
  gst_memory_map (mem, &info, GST_MAP_WRITE);

  if (queue_item == NULL) {
    memset (info.data, 0x00, size);
  } else {
    std::memcpy(info.data, queue_item, size);
  }

  gst_memory_unmap (mem, &info);
  gst_buffer_append_memory (buf, mem);

  *buffer = buf;

  GST_DEBUG_OBJECT (rossrc, "Buffer of TensorRosSrc is pushed! (queue size: %d)\n",
    g_async_queue_length (rossrc->queue));

  return GST_FLOW_OK;
}

 /**
 * @brief Initialize the tensor_ros_src plugin
 */
static gboolean
gst_tensor_ros_src_plugin_init (GstPlugin * rossrc)
{
  GST_DEBUG_CATEGORY_INIT (gst_tensor_ros_src_debug, "tensor_ros_src",
      0, "Source element to get the topic data from ROS node");

  return gst_element_register (rossrc, "tensor_ros_src", GST_RANK_NONE,
      GST_TYPE_TENSOR_ROS_SRC);
}

#ifndef PACKAGE
#define PACKAGE "nnstreamer-ros"
#endif

/**
 * @brief Macro to define the entry point of the plugin.
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    tensor_ros_src,
    "Source element to get the topic data from ROS node",
    gst_tensor_ros_src_plugin_init,
    VERSION,
    "LGPL",
    "nnstreamer-ros",
    "https://github.com/nnsuite/nnstreamer-ros"
)