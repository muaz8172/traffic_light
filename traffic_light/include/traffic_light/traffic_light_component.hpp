#ifndef TRAFFIC_LIGHT__TRAFFIC_LIGHT_COMPONENT_HPP_
#define TRAFFIC_LIGHT__TRAFFIC_LIGHT_COMPONENT_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <traffic_light_msgs/action/lookup_traffic_light.hpp>
#include <traffic_light_msgs/msg/traffic_light.hpp>

namespace traffic_light
{

class TrafficLightComponent : public rclcpp::Node
{
public:
  explicit TrafficLightComponent(
    const rclcpp::NodeOptions & options);

  ~TrafficLightComponent() override;

private:
  using LookupTrafficLight =
    traffic_light_msgs::action::LookupTrafficLight;

  using GoalHandleLookup =
    rclcpp_action::ServerGoalHandle<LookupTrafficLight>;

  struct HsvRange
  {
    cv::Scalar lower;
    cv::Scalar upper;
  };

  //
  // One round candidate surviving the area, circularity, and radius
  // filters. Centre and radius are in pixels of the processed region;
  // they are shifted into full-image coordinates before publication.
  //
  struct Detection
  {
    double center_x{0.0};
    double center_y{0.0};
    double radius{0.0};
    double circularity{0.0};
    double area{0.0};
  };

  //
  // Fixed colour ordering shared by the range table, the detection
  // buffers, and the per-colour publishers.
  //
  static constexpr std::size_t kRed = 0U;
  static constexpr std::size_t kYellow = 1U;
  static constexpr std::size_t kGreen = 2U;
  static constexpr std::size_t kColorCount = 3U;

  using DetectionsByColor =
    std::array<std::vector<Detection>, kColorCount>;

  //
  // Everything one input image yields. The debug image is empty unless
  // debug image publication is enabled.
  //
  struct ProcessedFrame
  {
    traffic_light_msgs::msg::TrafficLight state;
    DetectionsByColor detections;
    cv::Rect region;
    cv::Mat debug_image;
  };

  static const char * color_name(std::size_t color);
  static cv::Scalar color_draw_bgr(std::size_t color);
  static std::uint8_t color_state(std::size_t color);

  void declare_parameters();
  bool load_parameters();

  bool load_hsv_range(
    const std::string & lower_parameter,
    const std::string & upper_parameter,
    HsvRange & range);

  //
  // Live retuning. The detection knobs and HSV ranges take effect on
  // the next frame; topic names, QoS depths, and the action name are
  // rejected because they would require rebuilding the endpoints.
  //
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  static bool valid_kernel_size(std::int64_t value);
  static bool valid_hsv_triplet(const std::vector<std::int64_t> & value);
  static bool valid_ratio(double value);

  void run();
  void stop();

  void image_callback(
    sensor_msgs::msg::Image::ConstSharedPtr image);

  ProcessedFrame process(
    const sensor_msgs::msg::Image::ConstSharedPtr & image) const;

  std::optional<cv::Rect> resolve_region(
    int image_width,
    int image_height) const;

  cv::Mat create_mask(
    const cv::Mat & hsv,
    const HsvRange & range) const;

  std::vector<Detection> find_round_objects(
    const cv::Mat & mask) const;

  void annotate(
    cv::Mat & debug_image,
    const DetectionsByColor & detections,
    const cv::Rect & region) const;

  void publish_color_detections(
    std::size_t color,
    const std::vector<Detection> & detections,
    const cv::Rect & region,
    const sensor_msgs::msg::Image::ConstSharedPtr & image) const;

  void publish_debug_image(
    const cv::Mat & debug_image,
    const sensor_msgs::msg::Image::ConstSharedPtr & image) const;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const LookupTrafficLight::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleLookup> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleLookup> goal_handle);

  void lookup_timer_callback();

  std::shared_ptr<GoalHandleLookup> pending_lookup_at_image_start() const;

  void finish_lookup_from_image(
    const std::shared_ptr<GoalHandleLookup> & goal_handle,
    const traffic_light_msgs::msg::TrafficLight & traffic_light);

  traffic_light_msgs::msg::TrafficLight make_unknown_result() const;

  std::atomic<bool> stream_{true};

  std::string input_topic_{"image"};
  std::string output_topic_{"traffic_light/state"};
  std::string action_name_{"traffic_light/lookup"};

  bool publish_detections_{true};
  bool publish_debug_image_{true};

  std::array<std::string, kColorCount> detections_topics_{
    {"traffic_light/detections/red",
      "traffic_light/detections/yellow",
      "traffic_light/detections/green"}};

  std::string debug_image_topic_{"traffic_light/debug_image"};

  std::int64_t input_qos_depth_{1};
  std::int64_t output_qos_depth_{10};

  //
  // Everything below is retunable at runtime, so it is read and written
  // only under settings_mutex_. Image processing holds the lock for a
  // whole frame; a concurrent parameter set waits out that one frame.
  //
  mutable std::mutex settings_mutex_;

  double keep_top_ratio_{0.55};

  std::int64_t roi_x_{0};
  std::int64_t roi_y_{0};
  std::int64_t roi_width_{0};
  std::int64_t roi_height_{0};

  std::int64_t blur_kernel_size_{9};
  std::int64_t morphology_kernel_size_{5};

  double minimum_area_ratio_{0.002};

  double min_area_px_{150.0};
  double min_radius_px_{10.0};
  double min_circularity_{0.75};

  std::array<HsvRange, kColorCount> ranges_;
  HsvRange red_range_2_;

  cv::Mat morphology_kernel_;

  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  rclcpp::Publisher<
    traffic_light_msgs::msg::TrafficLight>::SharedPtr state_publisher_;

  std::array<
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr,
    kColorCount> detections_publishers_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;

  rclcpp_action::Server<LookupTrafficLight>::SharedPtr lookup_action_server_;
  rclcpp::TimerBase::SharedPtr lookup_timer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  mutable std::mutex lookup_mutex_;
  std::shared_ptr<GoalHandleLookup> pending_lookup_;
  std::chrono::steady_clock::time_point lookup_deadline_;
  bool lookup_reserved_{false};
};

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT__TRAFFIC_LIGHT_COMPONENT_HPP_
