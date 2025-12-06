---
sidebar_position: 6
---

# Chapter 5: Environments

## Introduction

Creating realistic environments is crucial for effective simulation. This chapter covers building worlds, adding objects, and creating scenarios for testing robots.

## World File Structure

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Ground Plane

```xml
<include>
  <uri>model://ground_plane</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

## Lighting

### Sun

```xml
<include>
  <uri>model://sun</uri>
</include>
```

### Custom Lights

```xml
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.5 0.1 -0.9</direction>
</light>
```

## Adding Models

### From Model Database

```xml
<include>
  <uri>model://cafe</uri>
  <pose>5 0 0 0 0 0</pose>
</include>
```

### Custom Models

```xml
<model name="my_object">
  <pose>0 0 1 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
```

## Terrain

### Heightmaps

```xml
<heightmap>
  <uri>file://media/materials/textures/heightmap_bowl.png</uri>
  <size>100 100 10</size>
  <pos>0 0 0</pos>
</heightmap>
```

## Obstacles and Scenarios

### Creating Obstacle Courses

```xml
<model name="obstacle1">
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

## Spawning Robots

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity my_robot \
  -file /path/to/robot.urdf
```

## Dynamic Objects

```xml
<model name="dynamic_box">
  <static>false</static>
  <link name="link">
    <inertial>
      <mass>1.0</mass>
    </inertial>
  </link>
</model>
```

## Environment Variables

- **GAZEBO_MODEL_PATH**: Search path for models
- **GAZEBO_RESOURCE_PATH**: Search path for resources

## Best Practices

- Start simple, add complexity gradually
- Use realistic scales
- Test with different lighting
- Organize world files
- Document your environments

## Exercises

1. Create a simple indoor environment
2. Add obstacles for navigation testing
3. Build a warehouse scenario

## Next Steps

Chapter 6 covers advanced simulation techniques and optimization.

