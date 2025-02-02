# ROS2 Humble Docker Development Environment

This repository provides a Docker-based development environment for ROS2 Humble, specifically configured for macOS users. It includes all necessary setup for ROS2 tutorials and development.

## Prerequisites

- macOS (Apple Silicon or Intel)
- Docker Desktop for Mac
- XQuartz (for GUI applications)

## Initial Setup

1. **Install Docker Desktop**
   - Download and install from [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop/)
   - Launch Docker Desktop and ensure it's running

2. **Install XQuartz (for GUI applications)**
   ```bash
   brew install --cask xquartz
   ```
   - Open XQuartz
   - Go to XQuartz > Preferences > Security
   - Check "Allow connections from network clients"
   - Restart XQuartz

3. **Clone and Setup**
   ```bash
   # Clone this repository
   git clone <repository-url>
   cd <repository-name>

   # Build the Docker image
   docker compose build

   # Start the container
   docker compose up -d

   # Enter the container
   docker compose exec ros2_dev bash
   ```

## Directory Structure

```
.
├── Dockerfile           # Container configuration
├── docker-compose.yml   # Development environment setup
├── .gitignore          # Build and cache exclusions
└── src/                # Your ROS2 packages go here
```

## Tutorial Guides

### 1. Environment Configuration
```bash
# Verify ROS2 environment
printenv | grep -i ROS

# Check ROS2 installation
ros2 --help
```

### 2. Turtlesim Tutorial
```bash
# Start turtlesim
ros2 run turtlesim turtlesim_node

# In another terminal (docker compose exec ros2_dev bash):
ros2 run turtlesim turtle_teleop_key
```

### 3. Understanding Nodes
```bash
# List all running nodes
ros2 node list

# Get information about a node
ros2 node info /turtlesim

# Run demo nodes
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

### 4. Understanding Topics
```bash
# List all topics
ros2 topic list

# Echo topic data
ros2 topic echo /chatter

# Publish to a topic
ros2 topic pub /chatter std_msgs/String "data: 'hello'"
```

### 5. Understanding Services
```bash
# List all services
ros2 service list

# Call a service (example with turtlesim)
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

### 6. Understanding Parameters
```bash
# List parameters
ros2 param list

# Get parameter value
ros2 param get /turtlesim background_r

# Set parameter value
ros2 param set /turtlesim background_r 150
```

### 7. Using rqt_console
```bash
# Launch rqt_console
ros2 run rqt_console rqt_console

# Launch rqt (includes all rqt tools)
rqt
```

### 8. Launching Multiple Nodes
```bash
# Example launch file (create in your package)
ros2 launch your_package your_launch_file.py

# Launch turtlesim with teleop
ros2 launch turtlesim multisim.launch.py
```

### 9. Recording and Playing Back Data
```bash
# Record data
ros2 bag record /topic_name

# Play back recorded data
ros2 bag play rosbag2_yyyy_mm_dd-hh_mm_ss
```

## Development Tasks (10-15)

### 10. Using Colcon to Build Packages
```bash
# Build all packages
cd /root/ros2_ws
colcon build

# Build specific package
colcon build --packages-select my_package

# Source the workspace
source install/setup.bash
```

### 11. Creating a Workspace
The workspace is already set up at `/root/ros2_ws` in the container with the following structure:
```
ros2_ws/
├── src/        # Source space (your packages go here)
├── build/      # Build space (created after building)
├── install/    # Install space (created after building)
└── log/        # Log space (created after building)
```

### 12. Creating a Package
```bash
# Create a Python package
cd /root/ros2_ws/src
ros2 pkg create --build-type ament_python my_package --dependencies rclpy

# Create a C++ package
ros2 pkg create --build-type ament_cmake cpp_package --dependencies rclcpp
```

### 13. Writing a Simple Publisher/Subscriber (Python)
```bash
# Create a package for your publisher/subscriber
cd /root/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub --dependencies rclpy

# Build your package
cd /root/ros2_ws
colcon build --packages-select py_pubsub

# Run your nodes
source install/setup.bash
ros2 run py_pubsub talker
ros2 run py_pubsub listener
```

### 14. Writing a Simple Service/Client (Python)
```bash
# Create a package for your service/client
cd /root/ros2_ws/src
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy

# Build your package
cd /root/ros2_ws
colcon build --packages-select py_srvcli

# Run your nodes
source install/setup.bash
ros2 run py_srvcli service
ros2 run py_srvcli client
```

### 15. Using Parameters in a Class (Python)
```bash
# Create a package for your parameter node
cd /root/ros2_ws/src
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy

# Build your package
cd /root/ros2_ws
colcon build --packages-select python_parameters

# Run your node
source install/setup.bash
ros2 run python_parameters param_node
```

## Troubleshooting

### GUI Applications Not Working
1. Ensure XQuartz is running
2. Allow network connections in XQuartz preferences
3. Restart XQuartz and your terminal
4. Try restarting the container:
   ```bash
   docker compose down
   docker compose up -d
   ```

### Build Issues
```bash
# Clean build
cd /root/ros2_ws
rm -rf build/ install/
colcon build
```

### Container Access
```bash
# Enter the container
docker compose exec ros2_dev bash

# View container logs
docker compose logs

# Restart container
docker compose restart
```

### Common Issues

1. **X11 Display Error**
   ```bash
   # On your host machine (macOS)
   xhost +localhost
   ```

2. **Package Not Found**
   ```bash
   # Source the workspace
   source /root/ros2_ws/install/setup.bash
   ```

3. **Permission Issues**
   ```bash
   # If you encounter permission issues with mounted volumes
   sudo chown -R $USER:$USER .
   ```

## Additional Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials/)
- [ROS2 Answers](https://answers.ros.org/)
- [Docker Documentation](https://docs.docker.com/)
- [ROS2 Design](https://design.ros2.org/)
- [ROS2 Index](https://index.ros.org/doc/ros2/)

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details 