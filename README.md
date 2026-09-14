# traffic_light packages

ROS 2 Humble packages for HSV-based traffic-light colour classification.

Round red, yellow, and green objects are located in the upper part of the camera image, published per colour with their pixel centre and radius, and reduced to a single traffic-light state.

- `traffic_light_msgs`: `TrafficLight` message and `LookupTrafficLight` action.
- `traffic_light`: composable component and standalone node with configurable input topic, state output topic, per-colour detection topics, annotated debug image, action name, top crop, ROI, HSV ranges, round-candidate filters, processing settings, and QoS depths. Detection settings are retunable at runtime.

See `traffic_light/README.md` for build, configuration, and usage.
