// src/traffic_light/traffic_light_component.cpp

#include "traffic_light/traffic_light_component.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <std_msgs/msg/header.hpp>

namespace traffic_light
{

namespace
{

//
// Used for published headers only when the input image carries no
// frame_id of its own.
//
constexpr const char * kDefaultFrameId = "camera_link";

//
// Defined locally because M_PI is not part of standard C++ and is
// hidden by strict-conformance builds.
//
constexpr double kPi = 3.14159265358979323846;

//
// Overlay styling. These are presentation details of the debug image,
// not detector behavior, so they are deliberately not parameters.
//
constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;

constexpr double kLabelScale = 0.5;
constexpr double kCoordinateScale = 0.45;
constexpr double kHeadingScale = 0.6;
constexpr double kCountScale = 0.55;

const cv::Scalar kCenterBgr(0.0, 0.0, 255.0);
const cv::Scalar kCrosshairBgr(255.0, 0.0, 0.0);
const cv::Scalar kTextBgr(255.0, 255.0, 255.0);

}  // namespace

const char * TrafficLightComponent::color_name(
  std::size_t color)
{
  switch (color) {
    case kRed:
      return "RED";

    case kYellow:
      return "YELLOW";

    default:
      return "GREEN";
  }
}

cv::Scalar TrafficLightComponent::color_draw_bgr(
  std::size_t color)
{
  switch (color) {
    case kRed:
      return cv::Scalar(0.0, 0.0, 255.0);

    case kYellow:
      return cv::Scalar(0.0, 255.0, 255.0);

    default:
      return cv::Scalar(0.0, 255.0, 0.0);
  }
}

std::uint8_t TrafficLightComponent::color_state(
  std::size_t color)
{
  switch (color) {
    case kRed:
      return traffic_light_msgs::msg::TrafficLight::RED;

    case kYellow:
      return traffic_light_msgs::msg::TrafficLight::YELLOW;

    default:
      return traffic_light_msgs::msg::TrafficLight::GREEN;
  }
}

TrafficLightComponent::TrafficLightComponent(
  const rclcpp::NodeOptions & options)
: rclcpp::Node(
    "traffic_light",
    options)
{
  declare_parameters();

  if (!load_parameters()) {
    throw std::invalid_argument(
            "Invalid traffic_light parameters");
  }

  run();
}

TrafficLightComponent::~TrafficLightComponent()
{
  stop();
}

void TrafficLightComponent::declare_parameters()
{
  declare_parameter<bool>(
    "stream",
    true);

  declare_parameter<std::string>(
    "input_topic",
    "image");

  declare_parameter<std::string>(
    "output_topic",
    "traffic_light/state");

  declare_parameter<std::string>(
    "action_name",
    "traffic_light/lookup");

  declare_parameter<bool>(
    "publish_detections",
    true);

  declare_parameter<bool>(
    "publish_debug_image",
    true);

  declare_parameter<std::string>(
    "detections_topic.red",
    "traffic_light/detections/red");

  declare_parameter<std::string>(
    "detections_topic.yellow",
    "traffic_light/detections/yellow");

  declare_parameter<std::string>(
    "detections_topic.green",
    "traffic_light/detections/green");

  declare_parameter<std::string>(
    "debug_image_topic",
    "traffic_light/debug_image");

  declare_parameter<std::int64_t>(
    "input_qos_depth",
    1);

  declare_parameter<std::int64_t>(
    "output_qos_depth",
    10);

  declare_parameter<double>(
    "keep_top_ratio",
    0.55);

  declare_parameter<std::int64_t>(
    "roi.x",
    0);

  declare_parameter<std::int64_t>(
    "roi.y",
    0);

  declare_parameter<std::int64_t>(
    "roi.width",
    0);

  declare_parameter<std::int64_t>(
    "roi.height",
    0);

  declare_parameter<std::int64_t>(
    "blur_kernel_size",
    9);

  declare_parameter<std::int64_t>(
    "morphology_kernel_size",
    5);

  declare_parameter<double>(
    "minimum_area_ratio",
    0.002);

  declare_parameter<double>(
    "min_area_px",
    150.0);

  declare_parameter<double>(
    "min_radius_px",
    10.0);

  declare_parameter<double>(
    "min_circularity",
    0.75);

  declare_parameter<std::vector<std::int64_t>>(
    "red.lower_1",
    {0, 120, 70});

  declare_parameter<std::vector<std::int64_t>>(
    "red.upper_1",
    {10, 255, 255});

  declare_parameter<std::vector<std::int64_t>>(
    "red.lower_2",
    {170, 120, 70});

  declare_parameter<std::vector<std::int64_t>>(
    "red.upper_2",
    {179, 255, 255});

  declare_parameter<std::vector<std::int64_t>>(
    "yellow.lower",
    {20, 100, 100});

  declare_parameter<std::vector<std::int64_t>>(
    "yellow.upper",
    {35, 255, 255});

  declare_parameter<std::vector<std::int64_t>>(
    "green.lower",
    {40, 60, 60});

  declare_parameter<std::vector<std::int64_t>>(
    "green.upper",
    {85, 255, 255});
}

bool TrafficLightComponent::load_parameters()
{
  stream_ =
    get_parameter("stream").as_bool();

  input_topic_ =
    get_parameter("input_topic").as_string();

  output_topic_ =
    get_parameter("output_topic").as_string();

  action_name_ =
    get_parameter("action_name").as_string();

  if (input_topic_.empty() ||
    output_topic_.empty() ||
    action_name_.empty())
  {
    RCLCPP_ERROR(
      get_logger(),
      "input_topic, output_topic, and action_name cannot be empty");

    return false;
  }

  if (input_topic_ == output_topic_) {
    RCLCPP_ERROR(
      get_logger(),
      "input_topic and output_topic must be different");

    return false;
  }

  publish_detections_ =
    get_parameter("publish_detections").as_bool();

  publish_debug_image_ =
    get_parameter("publish_debug_image").as_bool();

  detections_topics_[kRed] =
    get_parameter("detections_topic.red").as_string();

  detections_topics_[kYellow] =
    get_parameter("detections_topic.yellow").as_string();

  detections_topics_[kGreen] =
    get_parameter("detections_topic.green").as_string();

  debug_image_topic_ =
    get_parameter("debug_image_topic").as_string();

  if (publish_detections_) {
    for (std::size_t color = 0U; color < kColorCount; ++color) {
      if (detections_topics_[color].empty()) {
        RCLCPP_ERROR(
          get_logger(),
          "detections_topic.%s cannot be empty when publish_detections is true",
          color_name(color));

        return false;
      }
    }

    if (detections_topics_[kRed] == detections_topics_[kYellow] ||
      detections_topics_[kRed] == detections_topics_[kGreen] ||
      detections_topics_[kYellow] == detections_topics_[kGreen])
    {
      RCLCPP_ERROR(
        get_logger(),
        "detections_topic.red, .yellow, and .green must all be different");

      return false;
    }
  }

  if (publish_debug_image_ && debug_image_topic_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "debug_image_topic cannot be empty when publish_debug_image is true");

    return false;
  }

  if (publish_debug_image_ && debug_image_topic_ == input_topic_) {
    RCLCPP_ERROR(
      get_logger(),
      "debug_image_topic and input_topic must be different");

    return false;
  }

  input_qos_depth_ =
    get_parameter("input_qos_depth").as_int();

  output_qos_depth_ =
    get_parameter("output_qos_depth").as_int();

  keep_top_ratio_ =
    get_parameter("keep_top_ratio").as_double();

  roi_x_ =
    get_parameter("roi.x").as_int();

  roi_y_ =
    get_parameter("roi.y").as_int();

  roi_width_ =
    get_parameter("roi.width").as_int();

  roi_height_ =
    get_parameter("roi.height").as_int();

  blur_kernel_size_ =
    get_parameter("blur_kernel_size").as_int();

  morphology_kernel_size_ =
    get_parameter("morphology_kernel_size").as_int();

  minimum_area_ratio_ =
    get_parameter("minimum_area_ratio").as_double();

  min_area_px_ =
    get_parameter("min_area_px").as_double();

  min_radius_px_ =
    get_parameter("min_radius_px").as_double();

  min_circularity_ =
    get_parameter("min_circularity").as_double();

  if (input_qos_depth_ <= 0 || output_qos_depth_ <= 0) {
    RCLCPP_ERROR(
      get_logger(),
      "QoS depths must be greater than zero");

    return false;
  }

  if (!std::isfinite(keep_top_ratio_) ||
    keep_top_ratio_ <= 0.0 ||
    keep_top_ratio_ > 1.0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "keep_top_ratio must be in the range (0.0, 1.0]");

    return false;
  }

  if (roi_x_ < 0 || roi_y_ < 0 ||
    roi_width_ < 0 || roi_height_ < 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "ROI values cannot be negative");

    return false;
  }

  const bool full_region_roi =
    roi_x_ == 0 && roi_y_ == 0 &&
    roi_width_ == 0 && roi_height_ == 0;

  const bool fixed_roi =
    roi_width_ > 0 && roi_height_ > 0;

  if (!full_region_roi && !fixed_roi) {
    RCLCPP_ERROR(
      get_logger(),
      "ROI must be all zeros for the full cropped region, "
      "or have positive width and height");

    return false;
  }

  if (roi_x_ > std::numeric_limits<int>::max() ||
    roi_y_ > std::numeric_limits<int>::max() ||
    roi_width_ > std::numeric_limits<int>::max() ||
    roi_height_ > std::numeric_limits<int>::max())
  {
    RCLCPP_ERROR(
      get_logger(),
      "ROI values are too large");

    return false;
  }

  if (!valid_kernel_size(blur_kernel_size_)) {
    RCLCPP_ERROR(
      get_logger(),
      "blur_kernel_size must be a positive odd number");

    return false;
  }

  if (!valid_kernel_size(morphology_kernel_size_)) {
    RCLCPP_ERROR(
      get_logger(),
      "morphology_kernel_size must be a positive odd number");

    return false;
  }

  if (!std::isfinite(minimum_area_ratio_) ||
    minimum_area_ratio_ <= 0.0 ||
    minimum_area_ratio_ > 1.0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "minimum_area_ratio must be in the range (0.0, 1.0]");

    return false;
  }

  if (!std::isfinite(min_area_px_) || min_area_px_ <= 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "min_area_px must be greater than zero");

    return false;
  }

  if (!std::isfinite(min_radius_px_) || min_radius_px_ < 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "min_radius_px cannot be negative");

    return false;
  }

  if (!std::isfinite(min_circularity_) ||
    min_circularity_ <= 0.0 ||
    min_circularity_ > 1.0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "min_circularity must be in the range (0.0, 1.0]");

    return false;
  }

  if (!load_hsv_range(
      "red.lower_1",
      "red.upper_1",
      ranges_[kRed]) ||
    !load_hsv_range(
      "red.lower_2",
      "red.upper_2",
      red_range_2_) ||
    !load_hsv_range(
      "yellow.lower",
      "yellow.upper",
      ranges_[kYellow]) ||
    !load_hsv_range(
      "green.lower",
      "green.upper",
      ranges_[kGreen]))
  {
    return false;
  }

  if (morphology_kernel_size_ > 1) {
    const int kernel_size =
      static_cast<int>(morphology_kernel_size_);

    morphology_kernel_ = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(kernel_size, kernel_size));
  }

  return true;
}

bool TrafficLightComponent::load_hsv_range(
  const std::string & lower_parameter,
  const std::string & upper_parameter,
  HsvRange & range)
{
  const auto lower =
    get_parameter(lower_parameter).as_integer_array();

  const auto upper =
    get_parameter(upper_parameter).as_integer_array();

  if (lower.size() != 3U || upper.size() != 3U) {
    RCLCPP_ERROR(
      get_logger(),
      "%s and %s must each contain exactly three integers",
      lower_parameter.c_str(),
      upper_parameter.c_str());

    return false;
  }

  if (!valid_hsv_triplet(lower) || !valid_hsv_triplet(upper)) {
    RCLCPP_ERROR(
      get_logger(),
      "%s and %s must use H in [0, 179] and S/V in [0, 255]",
      lower_parameter.c_str(),
      upper_parameter.c_str());

    return false;
  }

  if (lower[0] > upper[0] ||
    lower[1] > upper[1] ||
    lower[2] > upper[2])
  {
    RCLCPP_ERROR(
      get_logger(),
      "%s cannot be greater than %s",
      lower_parameter.c_str(),
      upper_parameter.c_str());

    return false;
  }

  range.lower = cv::Scalar(
    static_cast<double>(lower[0]),
    static_cast<double>(lower[1]),
    static_cast<double>(lower[2]));

  range.upper = cv::Scalar(
    static_cast<double>(upper[0]),
    static_cast<double>(upper[1]),
    static_cast<double>(upper[2]));

  return true;
}

bool TrafficLightComponent::valid_kernel_size(
  std::int64_t value)
{
  return value > 0 &&
         value <= std::numeric_limits<int>::max() &&
         value % 2 == 1;
}

bool TrafficLightComponent::valid_hsv_triplet(
  const std::vector<std::int64_t> & value)
{
  return value.size() == 3U &&
         value[0] >= 0 && value[0] <= 179 &&
         value[1] >= 0 && value[1] <= 255 &&
         value[2] >= 0 && value[2] <= 255;
}

bool TrafficLightComponent::valid_ratio(
  double value)
{
  return std::isfinite(value) && value > 0.0 && value <= 1.0;
}

rcl_interfaces::msg::SetParametersResult
TrafficLightComponent::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const auto incoming =
    [&parameters](const std::string & name) -> const rclcpp::Parameter *
    {
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == name) {
          return &parameter;
        }
      }

      return nullptr;
    };

  //
  // The effective value of a parameter for this request: the incoming
  // one when it is part of the same set, otherwise the current one.
  // This lets a single set command change both halves of an HSV range.
  //
  const auto effective_double =
    [&](const std::string & name)
    {
      const auto * parameter = incoming(name);

      return parameter ?
             parameter->as_double() :
             get_parameter(name).as_double();
    };

  const auto effective_int =
    [&](const std::string & name)
    {
      const auto * parameter = incoming(name);

      return parameter ?
             parameter->as_int() :
             get_parameter(name).as_int();
    };

  const auto effective_array =
    [&](const std::string & name)
    {
      const auto * parameter = incoming(name);

      return parameter ?
             parameter->as_integer_array() :
             get_parameter(name).as_integer_array();
    };

  const auto reject =
    [&result](const std::string & reason)
    {
      result.successful = false;
      result.reason = reason;
    };

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == "input_topic" ||
      name == "output_topic" ||
      name == "action_name" ||
      name == "publish_detections" ||
      name == "publish_debug_image" ||
      name == "detections_topic.red" ||
      name == "detections_topic.yellow" ||
      name == "detections_topic.green" ||
      name == "debug_image_topic" ||
      name == "input_qos_depth" ||
      name == "output_qos_depth")
    {
      reject(
        name +
        " cannot be changed at runtime; restart the node to apply it");

      return result;
    }
  }

  const double keep_top_ratio =
    effective_double("keep_top_ratio");

  if (!valid_ratio(keep_top_ratio)) {
    reject("keep_top_ratio must be in the range (0.0, 1.0]");
    return result;
  }

  const std::int64_t roi_x = effective_int("roi.x");
  const std::int64_t roi_y = effective_int("roi.y");
  const std::int64_t roi_width = effective_int("roi.width");
  const std::int64_t roi_height = effective_int("roi.height");

  if (roi_x < 0 || roi_y < 0 || roi_width < 0 || roi_height < 0) {
    reject("ROI values cannot be negative");
    return result;
  }

  const bool full_region_roi =
    roi_x == 0 && roi_y == 0 &&
    roi_width == 0 && roi_height == 0;

  const bool fixed_roi =
    roi_width > 0 && roi_height > 0;

  if (!full_region_roi && !fixed_roi) {
    reject(
      "ROI must be all zeros for the full cropped region, "
      "or have positive width and height");

    return result;
  }

  if (roi_x > std::numeric_limits<int>::max() ||
    roi_y > std::numeric_limits<int>::max() ||
    roi_width > std::numeric_limits<int>::max() ||
    roi_height > std::numeric_limits<int>::max())
  {
    reject("ROI values are too large");
    return result;
  }

  const std::int64_t blur_kernel_size =
    effective_int("blur_kernel_size");

  const std::int64_t morphology_kernel_size =
    effective_int("morphology_kernel_size");

  if (!valid_kernel_size(blur_kernel_size)) {
    reject("blur_kernel_size must be a positive odd number");
    return result;
  }

  if (!valid_kernel_size(morphology_kernel_size)) {
    reject("morphology_kernel_size must be a positive odd number");
    return result;
  }

  const double minimum_area_ratio =
    effective_double("minimum_area_ratio");

  const double min_area_px =
    effective_double("min_area_px");

  const double min_radius_px =
    effective_double("min_radius_px");

  const double min_circularity =
    effective_double("min_circularity");

  if (!valid_ratio(minimum_area_ratio)) {
    reject("minimum_area_ratio must be in the range (0.0, 1.0]");
    return result;
  }

  if (!std::isfinite(min_area_px) || min_area_px <= 0.0) {
    reject("min_area_px must be greater than zero");
    return result;
  }

  if (!std::isfinite(min_radius_px) || min_radius_px < 0.0) {
    reject("min_radius_px cannot be negative");
    return result;
  }

  if (!valid_ratio(min_circularity)) {
    reject("min_circularity must be in the range (0.0, 1.0]");
    return result;
  }

  const std::array<std::pair<std::string, std::string>, 4> range_names{
    {{"red.lower_1", "red.upper_1"},
      {"red.lower_2", "red.upper_2"},
      {"yellow.lower", "yellow.upper"},
      {"green.lower", "green.upper"}}};

  std::array<HsvRange, 4> staged_ranges;

  for (std::size_t index = 0U; index < range_names.size(); ++index) {
    const auto & lower_name = range_names[index].first;
    const auto & upper_name = range_names[index].second;

    const auto lower = effective_array(lower_name);
    const auto upper = effective_array(upper_name);

    if (!valid_hsv_triplet(lower) || !valid_hsv_triplet(upper)) {
      reject(
        lower_name + " and " + upper_name +
        " must each contain three values with H in [0, 179] "
        "and S/V in [0, 255]");

      return result;
    }

    if (lower[0] > upper[0] ||
      lower[1] > upper[1] ||
      lower[2] > upper[2])
    {
      reject(lower_name + " cannot be greater than " + upper_name);
      return result;
    }

    staged_ranges[index].lower = cv::Scalar(
      static_cast<double>(lower[0]),
      static_cast<double>(lower[1]),
      static_cast<double>(lower[2]));

    staged_ranges[index].upper = cv::Scalar(
      static_cast<double>(upper[0]),
      static_cast<double>(upper[1]),
      static_cast<double>(upper[2]));
  }

  cv::Mat morphology_kernel;

  if (morphology_kernel_size > 1) {
    const int kernel_size =
      static_cast<int>(morphology_kernel_size);

    morphology_kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(kernel_size, kernel_size));
  }

  //
  // Everything validated; commit as one batch so a frame never sees a
  // half-applied retune.
  //
  {
    std::lock_guard<std::mutex> lock(settings_mutex_);

    keep_top_ratio_ = keep_top_ratio;

    roi_x_ = roi_x;
    roi_y_ = roi_y;
    roi_width_ = roi_width;
    roi_height_ = roi_height;

    blur_kernel_size_ = blur_kernel_size;
    morphology_kernel_size_ = morphology_kernel_size;

    minimum_area_ratio_ = minimum_area_ratio;
    min_area_px_ = min_area_px;
    min_radius_px_ = min_radius_px;
    min_circularity_ = min_circularity;

    ranges_[kRed] = staged_ranges[0];
    red_range_2_ = staged_ranges[1];
    ranges_[kYellow] = staged_ranges[2];
    ranges_[kGreen] = staged_ranges[3];

    morphology_kernel_ = morphology_kernel;
  }

  if (const auto * parameter = incoming("stream")) {
    stream_ = parameter->as_bool();
  }

  return result;
}

void TrafficLightComponent::run()
{
  image_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  action_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto input_qos =
    rclcpp::SensorDataQoS().keep_last(
    static_cast<std::size_t>(input_qos_depth_));

  const auto output_qos =
    rclcpp::QoS(
    rclcpp::KeepLast(
      static_cast<std::size_t>(output_qos_depth_)))
    .reliable()
    .durability_volatile();

  state_publisher_ =
    create_publisher<traffic_light_msgs::msg::TrafficLight>(
    output_topic_,
    output_qos);

  if (publish_detections_) {
    for (std::size_t color = 0U; color < kColorCount; ++color) {
      detections_publishers_[color] =
        create_publisher<geometry_msgs::msg::PoseArray>(
        detections_topics_[color],
        output_qos);
    }
  }

  if (publish_debug_image_) {
    debug_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(
      debug_image_topic_,
      output_qos);
  }

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = image_callback_group_;

  image_subscription_ =
    create_subscription<sensor_msgs::msg::Image>(
    input_topic_,
    input_qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr image)
    {
      image_callback(std::move(image));
    },
    subscription_options);

  lookup_action_server_ =
    rclcpp_action::create_server<LookupTrafficLight>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    action_name_,
    std::bind(
      &TrafficLightComponent::handle_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &TrafficLightComponent::handle_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &TrafficLightComponent::handle_accepted,
      this,
      std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_callback_group_);

  lookup_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    [this]()
    {
      lookup_timer_callback();
    },
    action_callback_group_);

  parameter_callback_handle_ =
    add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters)
    {
      return on_set_parameters(parameters);
    });
}

void TrafficLightComponent::stop()
{
  if (lookup_timer_) {
    lookup_timer_->cancel();
  }

  parameter_callback_handle_.reset();

  lookup_timer_.reset();
  lookup_action_server_.reset();
  image_subscription_.reset();
  state_publisher_.reset();
  debug_image_publisher_.reset();

  for (auto & publisher : detections_publishers_) {
    publisher.reset();
  }

  image_callback_group_.reset();
  action_callback_group_.reset();

  std::lock_guard<std::mutex> lock(lookup_mutex_);
  pending_lookup_.reset();
  lookup_reserved_ = false;
}

void TrafficLightComponent::image_callback(
  sensor_msgs::msg::Image::ConstSharedPtr image)
{
  const auto lookup_goal =
    pending_lookup_at_image_start();

  if (!stream_ && !lookup_goal) {
    return;
  }

  ProcessedFrame frame;

  try {
    frame = process(image);
  } catch (const cv_bridge::Exception & exception) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Image conversion failed: %s",
      exception.what());

    return;
  } catch (const cv::Exception & exception) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "OpenCV processing failed: %s",
      exception.what());

    return;
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Traffic-light processing failed: %s",
      exception.what());

    return;
  }

  //
  // Detections and the debug image are stream outputs, so they follow
  // the same rule as the state topic: nothing is published while
  // stream is false, even when an action goal drove the processing.
  //
  if (stream_) {
    state_publisher_->publish(frame.state);

    if (publish_detections_) {
      for (std::size_t color = 0U; color < kColorCount; ++color) {
        publish_color_detections(
          color,
          frame.detections[color],
          frame.region,
          image);
      }
    }

    if (publish_debug_image_ && !frame.debug_image.empty()) {
      publish_debug_image(
        frame.debug_image,
        image);
    }
  }

  if (lookup_goal) {
    finish_lookup_from_image(
      lookup_goal,
      frame.state);
  }
}

TrafficLightComponent::ProcessedFrame
TrafficLightComponent::process(
  const sensor_msgs::msg::Image::ConstSharedPtr & image) const
{
  const auto cv_image = cv_bridge::toCvShare(
    image,
    sensor_msgs::image_encodings::BGR8);

  //
  // Held for the whole frame so a live retune cannot land between the
  // crop and the masks. Image callbacks are mutually exclusive, so the
  // only contention is an occasional parameter set.
  //
  std::lock_guard<std::mutex> lock(settings_mutex_);

  const auto region = resolve_region(
    cv_image->image.cols,
    cv_image->image.rows);

  if (!region) {
    throw std::runtime_error(
            "Configured crop and ROI are outside the input image");
  }

  const cv::Mat bgr =
    cv_image->image(*region);

  //
  // Blur and the HSV conversion happen once and are shared by all three
  // colour masks.
  //
  cv::Mat filtered;

  if (blur_kernel_size_ > 1) {
    const int kernel_size =
      static_cast<int>(blur_kernel_size_);

    cv::GaussianBlur(
      bgr,
      filtered,
      cv::Size(kernel_size, kernel_size),
      0.0);
  } else {
    filtered = bgr;
  }

  cv::Mat hsv;
  cv::cvtColor(
    filtered,
    hsv,
    cv::COLOR_BGR2HSV);

  ProcessedFrame frame;

  frame.region = *region;
  frame.state.header = image->header;
  frame.state.state =
    traffic_light_msgs::msg::TrafficLight::UNKNOWN;

  const double region_area =
    static_cast<double>(region->area());

  double best_area = 0.0;
  std::size_t best_color = kRed;
  bool has_best = false;

  for (std::size_t color = 0U; color < kColorCount; ++color) {
    cv::Mat mask =
      create_mask(hsv, ranges_[color]);

    //
    // Red is the only hue that wraps past 179, so it gets a second band
    // merged in before the morphology pass.
    //
    if (color == kRed) {
      const cv::Mat wrap_mask =
        create_mask(hsv, red_range_2_);

      cv::bitwise_or(mask, wrap_mask, mask);
    }

    if (!morphology_kernel_.empty()) {
      cv::morphologyEx(
        mask,
        mask,
        cv::MORPH_OPEN,
        morphology_kernel_,
        cv::Point(-1, -1),
        1);

      cv::morphologyEx(
        mask,
        mask,
        cv::MORPH_CLOSE,
        morphology_kernel_,
        cv::Point(-1, -1),
        2);
    }

    frame.detections[color] = find_round_objects(mask);

    //
    // Detections are sorted by radius, but colour arbitration uses
    // contour area so that minimum_area_ratio keeps its original
    // meaning of "mask area divided by region area".
    //
    double color_area = 0.0;

    for (const auto & detection : frame.detections[color]) {
      color_area = std::max(color_area, detection.area);
    }

    if (color_area > best_area) {
      best_area = color_area;
      best_color = color;
      has_best = true;
    }
  }

  if (has_best &&
    region_area > 0.0 &&
    best_area / region_area >= minimum_area_ratio_)
  {
    frame.state.state = color_state(best_color);
  }

  if (publish_debug_image_) {
    frame.debug_image = bgr.clone();

    annotate(
      frame.debug_image,
      frame.detections,
      *region);
  }

  return frame;
}

std::optional<cv::Rect> TrafficLightComponent::resolve_region(
  int image_width,
  int image_height) const
{
  if (image_width <= 0 || image_height <= 0) {
    return std::nullopt;
  }

  //
  // The top crop is applied first; the ROI is then resolved inside the
  // cropped band. keep_top_ratio of 1.0 keeps the whole image.
  //
  int cropped_height = static_cast<int>(
    static_cast<double>(image_height) * keep_top_ratio_);

  cropped_height = std::max(1, std::min(cropped_height, image_height));

  if (roi_width_ == 0 && roi_height_ == 0) {
    return cv::Rect(
      0,
      0,
      image_width,
      cropped_height);
  }

  const int x = static_cast<int>(roi_x_);
  const int y = static_cast<int>(roi_y_);
  const int width = static_cast<int>(roi_width_);
  const int height = static_cast<int>(roi_height_);

  if (x >= image_width || y >= cropped_height ||
    width > image_width - x ||
    height > cropped_height - y)
  {
    return std::nullopt;
  }

  return cv::Rect(x, y, width, height);
}

cv::Mat TrafficLightComponent::create_mask(
  const cv::Mat & hsv,
  const HsvRange & range) const
{
  cv::Mat mask;

  cv::inRange(
    hsv,
    range.lower,
    range.upper,
    mask);

  return mask;
}

std::vector<TrafficLightComponent::Detection>
TrafficLightComponent::find_round_objects(
  const cv::Mat & mask) const
{
  std::vector<std::vector<cv::Point>> contours;

  //
  // findContours can consume its input, so it is given a copy.
  //
  cv::findContours(
    mask.clone(),
    contours,
    cv::RETR_EXTERNAL,
    cv::CHAIN_APPROX_SIMPLE);

  std::vector<Detection> detections;
  detections.reserve(contours.size());

  for (const auto & contour : contours) {
    const double area =
      cv::contourArea(contour);

    if (area < min_area_px_) {
      continue;
    }

    const double perimeter =
      cv::arcLength(contour, true);

    if (perimeter <= 0.0) {
      continue;
    }

    const double circularity =
      4.0 * kPi * area / (perimeter * perimeter);

    if (circularity < min_circularity_) {
      continue;
    }

    cv::Point2f center;
    float radius = 0.0F;

    cv::minEnclosingCircle(contour, center, radius);

    if (static_cast<double>(radius) < min_radius_px_) {
      continue;
    }

    Detection detection;
    detection.center_x = static_cast<double>(center.x);
    detection.center_y = static_cast<double>(center.y);
    detection.radius = static_cast<double>(radius);
    detection.circularity = circularity;
    detection.area = area;

    detections.push_back(detection);
  }

  //
  // Largest first, so index 0 is the main object for consumers that
  // only care about one.
  //
  std::sort(
    detections.begin(),
    detections.end(),
    [](const Detection & left, const Detection & right)
    {
      return left.radius > right.radius;
    });

  return detections;
}

void TrafficLightComponent::annotate(
  cv::Mat & debug_image,
  const DetectionsByColor & detections,
  const cv::Rect & region) const
{
  std::size_t total_count = 0U;

  for (std::size_t color = 0U; color < kColorCount; ++color) {
    const cv::Scalar draw_bgr = color_draw_bgr(color);

    total_count += detections[color].size();

    for (const auto & detection : detections[color]) {
      const cv::Point center(
        static_cast<int>(detection.center_x),
        static_cast<int>(detection.center_y));

      cv::circle(
        debug_image,
        center,
        static_cast<int>(detection.radius),
        draw_bgr,
        2);

      cv::circle(
        debug_image,
        center,
        4,
        kCenterBgr,
        -1);

      cv::line(
        debug_image,
        cv::Point(center.x - 10, center.y),
        cv::Point(center.x + 10, center.y),
        kCrosshairBgr,
        1);

      cv::line(
        debug_image,
        cv::Point(center.x, center.y - 10),
        cv::Point(center.x, center.y + 10),
        kCrosshairBgr,
        1);

      char label[96];

      std::snprintf(
        label,
        sizeof(label),
        "%s r=%.0fpx c=%.2f",
        color_name(color),
        detection.radius,
        detection.circularity);

      cv::putText(
        debug_image,
        label,
        cv::Point(center.x + 10, center.y - 10),
        kFont,
        kLabelScale,
        draw_bgr,
        1,
        cv::LINE_AA);

      //
      // The overlay reports the same full-image coordinates that are
      // published on the detection topics, not region-local ones.
      //
      char coordinates[64];

      std::snprintf(
        coordinates,
        sizeof(coordinates),
        "X:%.0f Y:%.0f",
        detection.center_x + static_cast<double>(region.x),
        detection.center_y + static_cast<double>(region.y));

      cv::putText(
        debug_image,
        coordinates,
        cv::Point(center.x + 10, center.y + 12),
        kFont,
        kCoordinateScale,
        kTextBgr,
        1,
        cv::LINE_AA);
    }
  }

  cv::putText(
    debug_image,
    "TOP CAMERA REGION",
    cv::Point(10, 25),
    kFont,
    kHeadingScale,
    kTextBgr,
    2,
    cv::LINE_AA);

  char count_text[96];

  std::snprintf(
    count_text,
    sizeof(count_text),
    "Objects: %zu  (R:%zu Y:%zu G:%zu)",
    total_count,
    detections[kRed].size(),
    detections[kYellow].size(),
    detections[kGreen].size());

  cv::putText(
    debug_image,
    count_text,
    cv::Point(10, 50),
    kFont,
    kCountScale,
    kTextBgr,
    2,
    cv::LINE_AA);
}

void TrafficLightComponent::publish_color_detections(
  std::size_t color,
  const std::vector<Detection> & detections,
  const cv::Rect & region,
  const sensor_msgs::msg::Image::ConstSharedPtr & image) const
{
  const auto & publisher = detections_publishers_[color];

  if (!publisher) {
    return;
  }

  geometry_msgs::msg::PoseArray pose_array;

  pose_array.header.stamp = image->header.stamp;

  if (!image->header.frame_id.empty()) {
    pose_array.header.frame_id = image->header.frame_id;
  } else {
    pose_array.header.frame_id = kDefaultFrameId;
  }

  pose_array.poses.reserve(detections.size());

  for (const auto & detection : detections) {
    geometry_msgs::msg::Pose pose;

    //
    // x and y are pixel coordinates in the full input image, z is the
    // detected radius in pixels. Orientation is unused and identity.
    //
    pose.position.x =
      detection.center_x + static_cast<double>(region.x);

    pose.position.y =
      detection.center_y + static_cast<double>(region.y);

    pose.position.z = detection.radius;

    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    pose_array.poses.push_back(pose);
  }

  publisher->publish(pose_array);
}

void TrafficLightComponent::publish_debug_image(
  const cv::Mat & debug_image,
  const sensor_msgs::msg::Image::ConstSharedPtr & image) const
{
  if (!debug_image_publisher_) {
    return;
  }

  std_msgs::msg::Header header;

  header.stamp = image->header.stamp;

  if (!image->header.frame_id.empty()) {
    header.frame_id = image->header.frame_id;
  } else {
    header.frame_id = kDefaultFrameId;
  }

  debug_image_publisher_->publish(
    *cv_bridge::CvImage(
      header,
      sensor_msgs::image_encodings::BGR8,
      debug_image).toImageMsg());
}

rclcpp_action::GoalResponse TrafficLightComponent::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const LookupTrafficLight::Goal> goal)
{
  if (!std::isfinite(goal->lookup_timeout) ||
    goal->lookup_timeout <= 0.0F)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::lock_guard<std::mutex> lock(lookup_mutex_);

  if (lookup_reserved_) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  lookup_reserved_ = true;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrafficLightComponent::handle_cancel(
  const std::shared_ptr<GoalHandleLookup>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrafficLightComponent::handle_accepted(
  const std::shared_ptr<GoalHandleLookup> goal_handle)
{
  const double timeout_seconds =
    static_cast<double>(
    goal_handle->get_goal()->lookup_timeout);

  const auto timeout =
    std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_seconds));

  std::lock_guard<std::mutex> lock(lookup_mutex_);

  pending_lookup_ = goal_handle;
  lookup_deadline_ =
    std::chrono::steady_clock::now() + timeout;
}

void TrafficLightComponent::lookup_timer_callback()
{
  std::shared_ptr<GoalHandleLookup> goal_handle;
  bool canceled = false;

  {
    std::lock_guard<std::mutex> lock(lookup_mutex_);

    if (!pending_lookup_) {
      return;
    }

    canceled = pending_lookup_->is_canceling();

    const bool timed_out =
      std::chrono::steady_clock::now() >= lookup_deadline_;

    if (!canceled && !timed_out) {
      return;
    }

    goal_handle = pending_lookup_;
    pending_lookup_.reset();
    lookup_reserved_ = false;
  }

  auto result =
    std::make_shared<LookupTrafficLight::Result>();

  result->traffic_light = make_unknown_result();

  if (canceled) {
    goal_handle->canceled(result);
  } else {
    goal_handle->abort(result);
  }
}

std::shared_ptr<TrafficLightComponent::GoalHandleLookup>
TrafficLightComponent::pending_lookup_at_image_start() const
{
  std::lock_guard<std::mutex> lock(lookup_mutex_);
  return pending_lookup_;
}

void TrafficLightComponent::finish_lookup_from_image(
  const std::shared_ptr<GoalHandleLookup> & goal_handle,
  const traffic_light_msgs::msg::TrafficLight & traffic_light)
{
  bool canceled = false;

  {
    std::lock_guard<std::mutex> lock(lookup_mutex_);

    if (pending_lookup_ != goal_handle) {
      return;
    }

    canceled = goal_handle->is_canceling();

    pending_lookup_.reset();
    lookup_reserved_ = false;
  }

  auto result =
    std::make_shared<LookupTrafficLight::Result>();

  result->traffic_light = traffic_light;

  if (canceled) {
    goal_handle->canceled(result);
  } else {
    goal_handle->succeed(result);
  }
}

traffic_light_msgs::msg::TrafficLight
TrafficLightComponent::make_unknown_result() const
{
  traffic_light_msgs::msg::TrafficLight result;
  result.header.stamp = now();
  result.state =
    traffic_light_msgs::msg::TrafficLight::UNKNOWN;

  return result;
}

}  // namespace traffic_light

RCLCPP_COMPONENTS_REGISTER_NODE(
  traffic_light::TrafficLightComponent)
