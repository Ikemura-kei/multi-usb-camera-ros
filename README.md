# Multi-USB Camera

## Configuration

Please write a YAML file for configuration.  

For example:

```
%YAML:1.0
---
camera0:
  name: "backward_left"
  rotate_angle: 180
  device_id: 0
  width: 800
  height: 480
  publish_image: true

camera1:
  name: "backward_right"
  rotate_angle: 180
  device_id: 2
  width: 800
  height: 480
  publish_image: false

camera2:
  name: "car_left"
  rotate_angle: 0
  device_id: 4
  width: 800
  height: 480
  publish_image: false
```

> Note: Please avoide any invalid characters that is not one of {a-z, A-Z, 0-9, / and _} for the camera name!