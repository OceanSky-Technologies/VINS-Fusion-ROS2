#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/app.h>

#include <thread>

class CamPublisherNode : public rclcpp::Node {
public:
  CamPublisherNode()
  : Node("cam_publisher_node")
  {
    // Initialize GStreamer
    gst_init(nullptr, nullptr);

    // Build the GStreamer pipeline using libcamerasrc
    // Force resolution 1456x1088 -> convert to RGB
    pipeline_ = gst_parse_launch(
      "libcamerasrc ! "
      "queue ! "
      "video/x-raw,format=RGBx,width=1456,height=1088 ! "
      "videoconvert ! "
      "video/x-raw,format=RGB ! "
      "appsink name=appsink0",
      nullptr
    );
    if (!pipeline_) {
      RCLCPP_ERROR(get_logger(), "Failed to create GStreamer pipeline!");
      return;
    }

    // Retrieve the appsink element by name
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink0");
    if (!appsink_) {
      RCLCPP_ERROR(get_logger(), "Failed to get 'appsink0' from pipeline!");
      return;
    }

    // Enable "new-sample" signal emission
    // "sync=false" means we don't block the pipeline if the app is slow
    g_object_set(G_OBJECT(appsink_), "emit-signals", TRUE, "sync", FALSE, NULL);

    // Connect our static callback
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample), this);

    // Set pipeline to PLAYING
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    // Create ROS2 publisher
    pub_ = create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", 10);

    // Create a GLib Main Loop and run it in a thread
    main_loop_ = g_main_loop_new(nullptr, FALSE);
    main_loop_thread_ = std::thread([this]() {
      g_main_loop_run(main_loop_);
    });

    RCLCPP_INFO(get_logger(), "Camera node started, publishing on '/cam0/image_raw'");
  }

  ~CamPublisherNode() override
  {
    // Stop the pipeline
    gst_element_set_state(pipeline_, GST_STATE_NULL);

    // Stop and free the GLib main loop
    g_main_loop_quit(main_loop_);
    if (main_loop_thread_.joinable()) {
      main_loop_thread_.join();
    }
    g_main_loop_unref(main_loop_);

    // Release pipeline resources
    if (appsink_) {
      gst_object_unref(appsink_);
    }
    if (pipeline_) {
      gst_object_unref(pipeline_);
    }
  }

private:
  // "new-sample" callback (static)
  static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data)
  {
    auto *node = static_cast<CamPublisherNode*>(user_data);

    // Pull sample from appsink
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
      return GST_FLOW_ERROR;  // No sample, pipeline error
    }

    // Access buffer (the actual frame data)
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    // Build a sensor_msgs::Image
    sensor_msgs::msg::Image msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = "camera_frame";

    // We forced 1456x1088, format=RGB (3 bytes/pixel)
    msg.width  = 1456;
    msg.height = 1088;
    msg.encoding = "rgb8";
    msg.step = msg.width * 3;

    // Copy the entire buffer into msg.data
    msg.data.assign(map.data, map.data + map.size);

    node->pub_->publish(msg);

    // Cleanup
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;  // Success
  }

  // ROS2 publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  // GStreamer objects
  GstElement *pipeline_ = nullptr;
  GstElement *appsink_ = nullptr;

  // GLib main loop for GStreamer signals
  GMainLoop *main_loop_ = nullptr;
  std::thread main_loop_thread_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}