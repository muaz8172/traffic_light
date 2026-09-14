# Node-RED integration

[`traffic_light_lookup_flow.json`](traffic_light_lookup_flow.json) calls the `traffic_light/lookup` action and turns the
answer into a LIMO `/cmd_vel` command. It targets [`@chart-sg/node-red-ros2`](https://github.com/chart-sg/node-red-ros2)
(current published version, `1.0.1`), a maintained fork of EduArt Robotik's `edu_nodered_ros2_plugin`.

## Install the palette

Run this on the machine that runs Node-RED, in a terminal where the workspace containing `traffic_light_msgs` is
already built and sourced — the palette shells out to `ros2 interface list` to populate its type pickers, and
`rclnodejs` needs to see the same environment to talk to your ROS graph:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash    # workspace containing traffic_light_msgs

cd ~/.node-red
npm install rclnodejs
npm install @chart-sg/node-red-ros2    # pulls in @chart-sg/node-red-ros2-manager automatically
```

Restart Node-RED from that same sourced terminal. `@chart-sg/node-red-ros2` does not install on native Windows
(`"os": ["!win32"]` in its `package.json`) — run Node-RED inside the same Ubuntu 22.04 / ROS 2 Humble environment
the `traffic_light` node itself targets (WSL2 or a Linux machine), not from a plain Windows shell.

## Import the flow

Node-RED menu → **Import** → paste the contents of `traffic_light_lookup_flow.json` (or drag the file in) → **Import to: new flow**.

The flow has two parts:

1. **Ask the traffic light for the current colour.** A `ROS2 Inject` node sends `{lookup_timeout: 1.0}` into an
   `Action Client` node targeting `/traffic_light/lookup`. Click the inject button to fire a lookup; the result
   comes back as `msg.payload.traffic_light.state` (`0`=UNKNOWN, `1`=RED, `2`=YELLOW, `3`=GREEN) and is split by a
   `switch` node into four debug outputs.
2. **Drive the LIMO based on the answer.** RED, YELLOW, and UNKNOWN all build a zero `Twist` (stop); GREEN builds a
   `linear.x: 0.2` `Twist` (go). Both feed a `Publisher` node on `/cmd_vel`.

## Things to match to your setup

- **Domain ID** — the flow's single `ros2-config` node ("Traffic Light Domain") defaults to domain `0`. Set it to
  whatever `ROS_DOMAIN_ID` your LIMO and the `traffic_light` node actually run on; every ROS2/RMF node in a
  Node-RED instance shares this one config node, so there is only one place to change it.
- **Action name** — `/traffic_light/lookup` must match the `action_name` parameter the node was launched with
  (default `traffic_light/lookup`, see [`traffic_light/config/traffic_light.yaml`](../traffic_light/config/traffic_light.yaml)).
- **`/cmd_vel`** — this is a bare demo (stop/go at a fixed speed). If your LIMO action client (see
  [`../examples/limo_traffic_light_action_client.py`](../examples/limo_traffic_light_action_client.py)) already owns
  driving decisions, don't run both against `/cmd_vel` at once — either let Node-RED publish, or let your action
  client publish, not both. A more realistic use of this flow is to publish the traffic-light state to a topic your
  existing robot logic already subscribes to (swap the two `function` + `Publisher` nodes for whatever that
  interface is), or to gate this flow's `Publisher` behind a `switch`/`filter` your navigation stack controls.
- **Repeated polling** — the `ROS2 Inject` node is set to manual (click-to-fire). To poll continuously instead,
  open it and set *Repeat* to an interval (e.g. every 1 second) — but remember `traffic_light/lookup` only accepts
  one goal at a time, so keep the interval longer than `lookup_timeout`.

If you'd rather stream state instead of polling on demand, skip the action entirely: a plain `Subscriber` node on
`traffic_light/state` (type `traffic_light_msgs/msg/TrafficLight`) gets you every classified frame without
round-tripping through an action goal each time.
