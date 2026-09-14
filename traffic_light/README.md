# traffic_light

ROS 2 Humble C++ component and standalone node that find round red, yellow, and green objects in the upper part of a camera image using HSV colour masks, and classify the frame as red, yellow, green, or unknown.

The package follows the `node_cpp_template` layout:

- normal `rclcpp::Node`
- composable component
- standalone executable
- multithreaded executor and separate callback groups
- parameter YAML
- Python and YAML launch files
- separate `traffic_light_msgs` interface package

## Interfaces

### Input

```text
Default: image
Type: sensor_msgs/msg/Image
Parameter: input_topic
```

The input uses sensor-data QoS: best effort, volatile, keep last. Remap `image` to the camera topic.

### Stream outputs

The classified state:

```text
Default: traffic_light/state
Type: traffic_light_msgs/msg/TrafficLight
Parameter: output_topic
```

Every round object found, one topic per colour:

```text
Default: traffic_light/detections/red
         traffic_light/detections/yellow
         traffic_light/detections/green
Type: geometry_msgs/msg/PoseArray
Parameters: detections_topic.red / .yellow / .green
Enabled by: publish_detections
```

A `PoseArray` has no field naming a colour, so each colour is split onto its own topic. Per pose:

```text
position.x  centre column in full-image pixels
position.y  centre row in full-image pixels
position.z  radius of the enclosing circle, in pixels
orientation identity, unused
```

Poses are ordered largest radius first, so `poses[0]` is the main object. Coordinates are shifted back into the **original** image frame, so the top crop and ROI offsets do not have to be undone downstream.

An annotated debug image:

```text
Default: traffic_light/debug_image
Type: sensor_msgs/msg/Image (bgr8)
Parameter: debug_image_topic
Enabled by: publish_debug_image
```

The debug image covers the processed region only, at its native size, with each detection circled and labelled. Masks remain internal processing data and are not published.

### On-demand action

```text
Default: traffic_light/lookup
Type: traffic_light_msgs/action/LookupTrafficLight
Parameter: action_name
```

Goal:

```text
float32 lookup_timeout
```

The timeout is in seconds and must be greater than zero.

The action waits for the first processable image received after the goal is accepted:

- a processed image returns `SUCCEEDED`, including when the detected state is `UNKNOWN`
- no processable image before `lookup_timeout` returns `ABORTED`
- client cancellation returns `CANCELED`
- only one lookup goal is accepted at a time

The result does not contain a separate success or status field because ROS action status already provides that information.

## Stream behavior

`stream: true`:

- every input image is processed
- one state message, one `PoseArray` per colour, and one debug image are published per successfully processed image
- an active action goal reuses the same processed result; the image is not processed twice

`stream: false`:

- incoming images are ignored when no action goal is active
- while a lookup goal is active, the next input image is processed once and returned
- no stream message is published, including detections and the debug image

Detections and the debug image are stream outputs and follow the same rule as the state topic. A lookup goal returns its result through the action, not through the stream topics.

## Detection algorithm

For each processed image, the node:

1. keeps the top `keep_top_ratio` of the image
2. selects the configured pixel ROI inside that band, or the whole band when the ROI is all zeros
3. applies optional Gaussian blur, once, shared by all three colours
4. converts BGR to HSV
5. builds a mask per colour, merging the two red bands before morphology
6. applies optional morphological opening once and closing twice
7. extracts external contours and keeps each one that passes `min_area_px`, `min_circularity`, and `min_radius_px`
8. sorts the survivors by radius, largest first, and publishes them per colour
9. chooses the colour whose largest contour is biggest, when it passes `minimum_area_ratio`
10. otherwise returns `UNKNOWN`

Circularity is `4 * pi * area / perimeter^2`, so a perfect circle scores `1.0`. Requiring a round shape rather than just a large blob is what separates a lit lamp from a red car door or a green hedge; the top crop does the rest, since an overhead traffic light never appears in the lower part of the frame.

A restricted ROI is still useful when the scene contains round coloured objects at traffic-light height, because HSV masking plus a shape test cannot distinguish those on its own.

Hue `11-19` is deliberately left out of every mask so that amber background objects trip neither red nor yellow. Widen `yellow.lower` down to `11` if orange should be classified as yellow.

## Parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `stream` | `true` | Enable continuous processing and stream publication. |
| `input_topic` | `image` | Input `sensor_msgs/msg/Image` topic. |
| `output_topic` | `traffic_light/state` | State output topic. |
| `action_name` | `traffic_light/lookup` | Action server name for on-demand lookup. |
| `publish_detections` | `true` | Publish the per-colour `PoseArray` topics. |
| `publish_debug_image` | `true` | Draw and publish the annotated debug image. |
| `detections_topic.red` | `traffic_light/detections/red` | Red detection topic. |
| `detections_topic.yellow` | `traffic_light/detections/yellow` | Yellow detection topic. |
| `detections_topic.green` | `traffic_light/detections/green` | Green detection topic. |
| `debug_image_topic` | `traffic_light/debug_image` | Annotated debug image topic. |
| `input_qos_depth` | `1` | Input image history depth. |
| `output_qos_depth` | `10` | Reliable stream output history depth. |
| `keep_top_ratio` | `0.55` | Fraction of image height kept from the top; `1.0` keeps all. |
| `roi.x` | `0` | ROI left pixel, inside the cropped band. |
| `roi.y` | `0` | ROI top pixel, inside the cropped band. |
| `roi.width` | `0` | ROI width. All ROI values zero means the whole cropped band. |
| `roi.height` | `0` | ROI height. |
| `blur_kernel_size` | `9` | Positive odd Gaussian kernel; `1` disables blur. |
| `morphology_kernel_size` | `5` | Positive odd morphology kernel; `1` disables morphology. |
| `min_area_px` | `150.0` | Minimum contour area in pixels. |
| `min_radius_px` | `10.0` | Minimum enclosing-circle radius in pixels. |
| `min_circularity` | `0.75` | Minimum `4*pi*area/perimeter^2`, in `(0.0, 1.0]`. |
| `minimum_area_ratio` | `0.002` | Minimum winning contour area divided by processed region area. |
| `red.lower_1` / `red.upper_1` | see YAML | Narrow low-hue red range. |
| `red.lower_2` / `red.upper_2` | see YAML | Narrow high-hue red wraparound range. |
| `yellow.lower` / `yellow.upper` | see YAML | Yellow range; orange is left unmatched by default. |
| `green.lower` / `green.upper` | see YAML | Green HSV range. |

OpenCV HSV uses hue `0..179` and saturation/value `0..255`.

## Build

Place both packages in the workspace `src` directory, then run:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select traffic_light_msgs traffic_light
source install/setup.bash
```

All required dependencies are declared for `rosdep`; there is no separate non-ROS dependency installer.

## Launch

Standalone node:

```bash
ros2 launch traffic_light traffic_light_node.launch.py
```

Composable component:

```bash
ros2 launch traffic_light traffic_light_component.launch.py
```

Example using the LIMO camera without a ROS remap:

```bash
ros2 run traffic_light traffic_light_node \
  --ros-args \
  --params-file "$(ros2 pkg prefix traffic_light)/share/traffic_light/config/traffic_light.yaml" \
  -p input_topic:=/limo/camera/color/image_raw
```

## Use

Read the stream:

```bash
ros2 topic echo /traffic_light/state
```

Request one lookup with a one-second timeout. The endpoint comes from `action_name`. `lookup_timeout` is not a node parameter; it is supplied independently in each action goal:

```bash
ros2 action send_goal \
  /traffic_light/lookup \
  traffic_light_msgs/action/LookupTrafficLight \
  "{lookup_timeout: 1.0}"
```

State constants:

```text
UNKNOWN = 0
RED     = 1
YELLOW  = 2
GREEN   = 3
```

Read the detected objects for one colour:

```bash
ros2 topic echo /traffic_light/detections/red
```

View the annotated debug image:

```bash
ros2 run rqt_image_view rqt_image_view /traffic_light/debug_image
```

## Tuning

HSV ranges and the round-candidate filters take effect on the next frame, so they can be adjusted against a live camera without restarting:

```bash
ros2 param set /traffic_light green.lower "[45, 80, 90]"
ros2 param set /traffic_light min_circularity 0.6
ros2 param set /traffic_light keep_top_ratio 0.4
```

Watch `/traffic_light/debug_image` while adjusting; each accepted candidate is drawn with its radius and circularity, so it is visible which filter is rejecting a lamp.

Live changes are validated as a batch and applied together, so a frame never sees a half-applied retune. A rejected set reports why and changes nothing:

```text
$ ros2 param set /traffic_light min_circularity 5.0
Setting parameter failed: min_circularity must be in the range (0.0, 1.0]
```

`stream`, `keep_top_ratio`, `roi.*`, the kernel sizes, the four filter thresholds, and all HSV ranges are retunable. Topic names, the action name, the QoS depths, and the `publish_*` flags are rejected at runtime because they would require rebuilding the endpoints; change them in the YAML and restart.

Traffic-light lamps are usually brighter and more saturated than a matte coloured object, so expect to raise the `v_min` component of a range and narrow its hue span once pointed at a real light.

## Configuration boundary

The public ROS interface names are parameters: `input_topic`, `output_topic`, `action_name`, the three `detections_topic.*` names, and `debug_image_topic`.
Detection ranges, the crop ratio, ROI, processing kernels, the round-candidate filters, stream behavior, and QoS depths are also parameters.

The 20 ms internal action-deadline check is intentionally not configurable. It is implementation timing, not part of the detector behavior, and exposing it would add a parameter without a practical operational need.

Debug-overlay styling — fonts, line thickness, and the colour each detection is drawn in — is likewise not configurable. It is presentation of a diagnostic topic, not detector behavior.

The node draws no GUI windows. Tuning happens through parameters and the debug image topic, which keeps the component headless-safe and safe to load into a shared component container.

## Quiet logging

Normal frame processing, state changes, detections, action acceptance, action completion, and lookup timeout do not print logs. The node logs only configuration errors and throttled image/OpenCV processing failures. Inspect detections on their topics or in the debug image rather than in the console.

## License

Apache License 2.0. See [LICENSE](LICENSE).
