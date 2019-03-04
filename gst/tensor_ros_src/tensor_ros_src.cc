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

#include "tensor_ros_src.h"

GST_DEBUG_CATEGORY_STATIC (gst_tensor_ros_src_debug);
#define GST_CAT_DEFAULT gst_tensor_ros_src_debug

#define SUPPORTED_CAPS_STRING "application/octet-stream"

/**
 * @brief tensor_ros_src properties
 */
enum
{
  PROP_0,
  PROP_SILENT,  /*<< Slient mode for debug */
  PROP_TOPIC,   /*<< ROS topic name to subscribe */
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
}

/**
 * @brief Dispose allocated resources
 */
static void
gst_tensor_ros_src_dispose (GObject * object)
{
  GstTensorRosSrc *src = GST_TENSOR_ROS_SRC (object);

  if (src->caps)
    gst_caps_unref (src->caps);

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
  GstTensorRosSrc *rossrc;
  GstBuffer *buf = NULL;
  GstMemory *mem;
  GstMapInfo info;
  gsize size;
  rossrc = GST_TENSOR_ROS_SRC (src);

  /* TODO: Need to fix real data */
  buf = gst_buffer_new ();
  size = 10 * tensor_element_size[_NNS_INT32];

  mem = gst_allocator_alloc (NULL, size, NULL);
  gst_memory_map (mem, &info, GST_MAP_WRITE);

  /* No need to initialize */
  memset (info.data, 0, size);
  gst_memory_unmap (mem, &info);
  gst_buffer_append_memory (buf, mem);

  *buffer = buf;

  GST_DEBUG_OBJECT (rossrc, "Buffer of TensorRosSrc is pushed! (size: %lu)\n", size);

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