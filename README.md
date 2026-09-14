# traffic_light

ROS 2 Humble packages that detect traffic-light colours in a camera image.

Round red, yellow, and green objects are located in the upper part of the frame, published per colour with their pixel centre and radius, and reduced to a single traffic-light state. Built for the LIMO robot's RGB camera, but nothing in it is LIMO-specific beyond the default topic in the examples.

- [`traffic_light`](traffic_light) — the detector: composable component, standalone node, parameters, and launch files.
- [`traffic_light_msgs`](traffic_light_msgs) — the `TrafficLight` message and `LookupTrafficLight` action.

---

## How it works

The detector is HSV colour masking plus a shape test. Colour alone is not enough — a red car door and a green hedge both survive a colour mask — so every candidate must also be *round*, and must appear where an overhead traffic light actually is.

Each incoming image goes through:

**1. Crop to the top of the frame.** Only the top `keep_top_ratio` (default `0.55`) is kept. A traffic light is above the road, never on it.

```text
┌──────────────────────────┐
│                          │
│        TOP 55%           │  ← processed
│                          │
├──────────────────────────┤
│     BOTTOM 45%           │  ← discarded
└──────────────────────────┘
```

An optional pixel ROI (`roi.*`) narrows this further, and is resolved *inside* the cropped band.

**2. Blur and convert to HSV**, once, shared by all three colours. HSV separates hue from brightness, so a lamp stays the same hue as daylight changes — which BGR does not.

**3. Build one mask per colour.** Red needs two hue bands, because red sits at both ends of the hue wheel (`0-10` and `170-179`); they are merged before the morphology step. Morphological opening then removes speckle, and closing fills holes inside a lamp.

**4. Keep only round contours.** Each external contour must pass three filters:

| Filter | Default | Rejects |
| --- | ---: | --- |
| `min_area_px` | `150` | specks and compression noise |
| `min_circularity` | `0.75` | elongated or ragged shapes — signs, reflections, foliage |
| `min_radius_px` | `10` | anything too small to be a lamp at working distance |

Circularity is `4πA/P²`, so a perfect circle scores `1.0`.

**5. Publish.** Survivors are sorted largest-radius-first and published per colour. The colour with the largest contour becomes the frame's state, provided it also passes `minimum_area_ratio`; otherwise the state is `UNKNOWN`.

There is no GUI. Tuning happens through ROS parameters and the annotated debug image topic, which keeps the component headless-safe and loadable into a shared component container.

---

## Requirements

- Ubuntu 22.04 with ROS 2 Humble
- A camera publishing `sensor_msgs/msg/Image`

OpenCV and `cv_bridge` come from `rosdep`; there is nothing to install by hand.

## Get it

Clone into the `src` directory of a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/muaz8172/traffic_light.git
```

## Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select traffic_light_msgs traffic_light
source install/setup.bash
```

Source `install/setup.bash` in every new terminal, or add it to your `~/.bashrc`.

## Update

To pull later changes and rebuild:

```bash
cd ~/ros2_ws/src/traffic_light
git pull
cd ~/ros2_ws
colcon build --symlink-install --packages-select traffic_light_msgs traffic_light
source install/setup.bash
```

If a build misbehaves after a pull, clear the stale artifacts first:

```bash
cd ~/ros2_ws
rm -rf build install log
```

## Run

Standalone node, pointed at the LIMO camera:

```bash
ros2 run traffic_light traffic_light_node \
  --ros-args \
  --params-file "$(ros2 pkg prefix traffic_light)/share/traffic_light/config/traffic_light.yaml" \
  -p input_topic:=/limo/camera/color/image_raw
```

Or via launch, which loads the same YAML:

```bash
ros2 launch traffic_light traffic_light_node.launch.py
```

As a composable component in a container:

```bash
ros2 launch traffic_light traffic_light_component.launch.py
```

## What it publishes

| Topic | Type | Contents |
| --- | --- | --- |
| `traffic_light/state` | `traffic_light_msgs/msg/TrafficLight` | `UNKNOWN`/`RED`/`YELLOW`/`GREEN` for the frame |
| `traffic_light/detections/red` | `geometry_msgs/msg/PoseArray` | every round red object |
| `traffic_light/detections/yellow` | `geometry_msgs/msg/PoseArray` | every round yellow object |
| `traffic_light/detections/green` | `geometry_msgs/msg/PoseArray` | every round green object |
| `traffic_light/debug_image` | `sensor_msgs/msg/Image` | processed region, each detection circled and labelled |

A `PoseArray` has no field naming a colour, so each colour gets its own topic. In each pose, `position.x` and `position.y` are the centre in full-image pixels, and `position.z` is the radius in pixels. Poses are ordered largest first, so `poses[0]` is the main object.

There is also an on-demand action, `traffic_light/lookup`, for asking "what colour is it right now?" and waiting for one answer instead of subscribing to the stream:

```bash
ros2 action send_goal /traffic_light/lookup \
  traffic_light_msgs/action/LookupTrafficLight "{lookup_timeout: 1.0}"
```

Watch the stream, or the detections for one colour:

```bash
ros2 topic echo /traffic_light/state
ros2 topic echo /traffic_light/detections/red
```

View the debug image:

```bash
ros2 run rqt_image_view rqt_image_view /traffic_light/debug_image
```

## Tuning

HSV ranges and the shape filters apply on the next frame, so they can be adjusted against a live camera without restarting the node:

```bash
ros2 param set /traffic_light green.lower "[45, 80, 90]"
ros2 param set /traffic_light min_circularity 0.6
ros2 param set /traffic_light keep_top_ratio 0.4
```

Watch `/traffic_light/debug_image` while adjusting — each accepted candidate is drawn with its radius and circularity, so it is visible which filter is rejecting a lamp. Once the values are right, copy them into [`traffic_light/config/traffic_light.yaml`](traffic_light/config/traffic_light.yaml) so they survive a restart.

Real traffic-light LEDs are far brighter and more saturated than a matte coloured object, so expect to raise the value floor of a range and narrow its hue span compared to the defaults.

Topic names, QoS depths, and the `publish_*` flags are rejected at runtime and need a restart; a rejected set reports why and changes nothing.

## Integrations

- [`nodered/`](nodered) — an importable Node-RED flow that calls `traffic_light/lookup` and drives a LIMO's
  `/cmd_vel` from the result, plus setup notes for the `@chart-sg/node-red-ros2` palette.
- [`examples/limo_traffic_light_action_client.py`](examples/limo_traffic_light_action_client.py) — a reference
  `rclpy` action client for `traffic_light/lookup`, meant to be folded into an existing LIMO control node.

## Repository layout

```text
traffic_light/            detector package
  config/                 traffic_light.yaml, all parameters
  include/, src/          component and standalone node
  launch/                 Python and YAML launch files
  README.md               full interface and parameter reference
traffic_light_msgs/       message and action definitions
nodered/                  Node-RED flow and setup notes
examples/                 reference LIMO action-client script
```

Full parameter tables, stream/action semantics, and the detection algorithm in detail are in [`traffic_light/README.md`](traffic_light/README.md).

## License

Apache License 2.0. See [LICENSE](traffic_light/LICENSE).
