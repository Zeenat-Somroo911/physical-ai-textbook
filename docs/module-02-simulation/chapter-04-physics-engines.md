---
sidebar_position: 5
---

# Chapter 4: Physics Engines

## Introduction

Physics engines simulate the physical behavior of objects in Gazebo. This chapter covers the available engines and how to configure them for optimal performance and accuracy.

## Available Physics Engines

Gazebo supports three physics engines:
- **ODE** (Open Dynamics Engine): Default, well-tested
- **Bullet**: Good for complex collisions
- **Simbody**: Accurate but slower

## Physics Engine Selection

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## Key Parameters

### Step Size

```xml
<max_step_size>0.001</max_step_size>
```
- Smaller = more accurate but slower
- Typical range: 0.0001 to 0.01

### Real-time Factor

```xml
<real_time_factor>1.0</real_time_factor>
```
- 1.0 = real-time
- Greater than 1.0 = faster than real-time
- Less than 1.0 = slower than real-time

### Update Rate

```xml
<real_time_update_rate>1000</real_time_update_rate>
```
- Higher = smoother but more CPU intensive
- Typical: 100-1000 Hz

## ODE Configuration

```xml
<physics type="ode">
  <solver>
    <type>quick</type>
    <iters>50</iters>
    <sor>1.3</sor>
    <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
  </solver>
  <constraints>
    <cfm>0.00001</cfm>
    <erp>0.2</erp>
    <contact_max_correcting_vel>100</contact_max_correcting_vel>
    <contact_surface_layer>0.001</contact_surface_layer>
  </constraints>
</physics>
```

## Collision Detection

### Contact Parameters

- **CFM** (Constraint Force Mixing): Softness
- **ERP** (Error Reduction Parameter): Constraint satisfaction
- **Surface Layer**: Penetration tolerance

### Friction

```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>
      <mu2>1.0</mu2>
    </ode>
  </friction>
</surface>
```

## Performance Optimization

### Reduce Collision Complexity

- Use simple collision geometries
- Limit number of contacts
- Use collision groups

### Tune Solver Iterations

```xml
<iters>50</iters>
```
- More iterations = more accurate but slower
- Find balance for your use case

## Gravity

```xml
<gravity>0 0 -9.81</gravity>
```

## Material Properties

```xml
<material>
  <ode>
    <mu>0.6</mu>
    <mu2>0.6</mu2>
    <fdir1>0 0 0</fdir1>
    <slip1>0</slip1>
    <slip2>0</slip2>
  </ode>
</material>
```

## Debugging Physics

- Enable visualization of contacts
- Check solver convergence
- Monitor real-time factor
- Profile performance

## Best Practices

- Start with default settings
- Tune for your specific robot
- Balance accuracy vs performance
- Test with realistic scenarios

## Exercises

1. Experiment with different step sizes
2. Tune friction parameters
3. Optimize for your robot

## Next Steps

Chapter 5 covers building simulation environments and worlds.

