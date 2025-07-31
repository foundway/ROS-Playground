# Docker Setup for ROS2 Humble Development

This directory contains Docker configuration files for setting up a ROS2 Humble development environment.

## Files

- `Dockerfile`: Defines the ROS2 Humble development container
- `docker-compose.yml`: Orchestrates the container setup and networking
- `README.md`: This file with usage instructions

## Quick Start

### Build and run the container:

```bash
# From the project root directory
cd docker
docker-compose up -d

# Or to run in foreground with interactive shell
docker-compose run --rm ros2-humble
```

### Access the running container:

```bash
docker exec -it ros2-humble-dev bash
```

### Stop the container:

```bash
docker-compose down
```

## Features

- **ROS2 Jazzy**: Full ROS2 Jazzy installation
- **Development Tools**: Git, vim, nano, curl, wget
- **Volume Mounting**: Your workspace is mounted for live development
- **noVNC Support**: GUI applications work via web browser (rqt, rviz, etc.)
- **Network Access**: Custom x11 network for ROS2 communication
- **Multi-robot Support**: Optional secondary container for multi-robot scenarios

## Usage Examples

### Basic ROS2 commands:

```bash
# Inside the container
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 node list
```

### Build your workspace:

```bash
# Inside the container
cd /ros2_ws
colcon build
source install/setup.bash
```

### Run with GUI support (noVNC):

```bash
# Run container with noVNC
docker-compose up -d

# Open your web browser and go to:
# http://localhost:8080

# Run GUI applications inside the container
docker exec -it jazzy rviz2
```

### Run with GUI support (Linux):

```bash
# Allow X11 connections
xhost +local:docker

# Run container
docker-compose up -d

# Run GUI applications
docker exec -it ros2-humble-dev rqt
```

### Multi-robot simulation:

```bash
# Start both containers
docker-compose --profile multi-robot up -d

# Access primary container
docker exec -it ros2-humble-dev bash

# Access secondary container
docker exec -it ros2-humble-secondary bash
```

## Configuration

### Environment Variables

- `ROS_DOMAIN_ID=0`: ROS2 domain ID for communication
- `ROS_LOCALHOST_ONLY=0`: Allow external ROS2 communication
- `DISPLAY=novnc:0`: noVNC display for GUI applications

### Ports

- `8080`: noVNC web interface
- `11311`: ROS Master port
- `7400-7500`: ROS2 DDS communication ports

### Volumes

- `../main_ws:/main_ws`: Mounts your workspace for live development

## Troubleshooting

### noVNC Connection Issues:
- Ensure port 8080 is not blocked by firewall
- Open http://localhost:8080 in your web browser
- Try refreshing the page if connection fails

### GUI Application Issues:
- Make sure you're connected via noVNC before running GUI apps
- Use `DISPLAY=novnc:0` for GUI applications

### Network Issues:
The container uses custom x11 network. Ensure your firewall allows ROS2 communication.

### Permission Issues:
The container runs with privileged mode. For production, consider using specific capabilities instead. 