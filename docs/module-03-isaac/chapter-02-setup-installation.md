---
sidebar_position: 3
---

# Chapter 2: Setup and Installation

## Introduction

This chapter guides you through installing and configuring NVIDIA Isaac Sim on your system.

## Installation Methods

### Docker (Recommended)

```bash
# Pull Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run Isaac Sim
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -v ~/docker/isaac-sim:/workspace \
  -v ~/docker/isaac-sim/cache:/root/.nvidia-omniverse/cache \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### Native Installation

1. Download Isaac Sim from NVIDIA Omniverse
2. Extract and run the installer
3. Follow the setup wizard
4. Configure environment variables

## Environment Setup

### Required Environment Variables

```bash
export ISAAC_SIM_PATH=/path/to/isaac-sim
export PYTHONPATH=$ISAAC_SIM_PATH:$PYTHONPATH
```

### Python Environment

```bash
# Create virtual environment
python3 -m venv isaac-sim-env
source isaac-sim-env/bin/activate

# Install Isaac Sim Python packages
pip install isaac-sim
```

## Verification

### Test Installation

```python
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

# Your code here

simulation_app.close()
```

## Configuration

### Graphics Settings

Configure in `~/.nvidia-omniverse/config/isaac-sim/settings.json`:

```json
{
  "renderer": "RayTracedLighting",
  "rtx": true,
  "max_fps": 60
}
```

### Physics Settings

```python
import omni.physx

# Configure physics
physx_interface = omni.physx.get_physx_interface()
physx_interface.set_gravity(0, 0, -9.81)
```

## Troubleshooting

### Common Issues

- **GPU not detected**: Check NVIDIA drivers
- **Import errors**: Verify PYTHONPATH
- **Performance issues**: Adjust graphics settings
- **Memory errors**: Reduce scene complexity

### Getting Help

- Isaac Sim Documentation
- NVIDIA Forums
- GitHub Issues
- Community Discord

## Next Steps

Chapter 3 covers scene composition and building environments.

