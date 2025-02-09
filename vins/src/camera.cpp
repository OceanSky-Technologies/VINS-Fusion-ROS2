#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// GStreamer headers
#include <gst/gst.h>
// #include <gst/app/app.h>

// TODO: fix path
#include "/usr/include/gstreamer-1.0/gst/gst.h"
#include "/usr/include/gstreamer-1.0/gst/app/app.h"

// A skeletal example
class CamPublisherNode : public rclcpp::Node {
public:
  CamPublisherNode() : Node("cam_publisher_node") {
    // Initialize GStreamer, build the pipeline...
    gst_init(nullptr, nullptr);
    pipeline_ = gst_parse_launch(
      "libcamerasrc ! "
      "video/x-raw,format=NV12,width=1456,height=1088,framerate=30/1 ! "
      "videoconvert ! "
      "video/x-raw,format=RGB ! "
      "appsink name=appsink0",
      nullptr);

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink0");
    g_object_set(G_OBJECT(appsink_), "emit-signals", TRUE, "sync", FALSE, NULL);
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(onNewSample), this);

    // Start pipeline
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    // Create ROS2 publisher
    pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
  }

  ~CamPublisherNode() {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }

private:
  static GstFlowReturn onNewSample(GstAppSink *appsink, gpointer user_data) {
    auto *node = static_cast<CamPublisherNode*>(user_data);
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    // Build a sensor_msgs/Image from map.data
    sensor_msgs::msg::Image msg;
    msg.height = /* your negotiated height, e.g. 1088 */;
    msg.width  = /* your negotiated width, e.g. 1456 */;
    // In this pipeline, format=RGB => 3 bytes/pixel
    msg.encoding = "rgb8";
    msg.step     = msg.width * 3;

    // Fill msg.data
    msg.data.assign(map.data, map.data + map.size);

    // Publish
    node->pub_->publish(msg);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  GstElement *pipeline_;
  GstElement *appsink_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CamPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
