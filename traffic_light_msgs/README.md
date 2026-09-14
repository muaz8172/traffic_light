# traffic_light_msgs

ROS 2 interfaces used by the `traffic_light` package.

## Message

`TrafficLight.msg` contains the source image header and one state:

- `UNKNOWN`
- `RED`
- `YELLOW`
- `GREEN`

`UNKNOWN` already represents that no configured colour passed the detection threshold, so no extra `detected` field is needed.

## Action

`LookupTrafficLight.action` accepts `lookup_timeout` in seconds and returns one `TrafficLight` result. The action status itself reports succeeded, canceled, or aborted, so the result does not duplicate action status.
