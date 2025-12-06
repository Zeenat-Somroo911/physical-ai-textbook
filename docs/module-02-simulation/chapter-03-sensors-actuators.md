---
sidebar_position: 4
---

# Chapter 3: Sensors and Actuators

## Introduction

Sensors and actuators are crucial for robot operation. This chapter covers how to configure and use various sensors and actuators in Gazebo.

## Camera Sensors

### Adding a Camera

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## LiDAR Sensors

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="head_lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>40</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## IMU Sensors

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/imu</namespace>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
      </angular_velocity>
    </imu>
  </sensor>
</gazebo>
```

## Joint Actuators

### Position Control

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find my_robot)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### Velocity Control

```xml
<joint name="wheel_joint">
  <command_interface name="velocity">
    <param name="min">-10</param>
    <param name="max">10</param>
  </command_interface>
</joint>
```

## ROS2 Integration

Sensors publish to ROS2 topics:
- Camera: `/camera/image_raw`
- LiDAR: `/lidar/scan`
- IMU: `/imu/data`

Actuators subscribe to commands:
- Joint position: `/joint_states`
- Velocity commands: `/cmd_vel`

## Sensor Noise

```xml
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```

## Best Practices

- Match sensor specs to real hardware
- Add realistic noise
- Test sensor data processing
- Optimize update rates

## Exercises

1. Add a camera to your robot model
2. Configure a LiDAR sensor
3. Set up joint controllers

## Next Steps

Chapter 4 explores physics engines and their configuration.

