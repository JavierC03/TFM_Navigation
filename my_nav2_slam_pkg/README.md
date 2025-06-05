# My Navigation and SLAM Package

This package integrates the Navigation 2 (Nav2) stack with Simultaneous Localization and Mapping (SLAM) capabilities for mobile robots. It provides a complete solution for autonomous navigation in unknown environments.

## Package Structure

- **config/**: Contains configuration files for Nav2 and SLAM.
  - `nav2_params.yaml`: Parameters for the Nav2 stack.
  - `slam_params.yaml`: Parameters for the SLAM algorithm.
  - `rviz_config.rviz`: RViz visualization settings.

- **launch/**: Contains launch files to start the navigation and SLAM systems.
  - `nav2_launch.py`: Launches the Nav2 stack.
  - `slam_launch.py`: Launches the SLAM nodes.
  - `navigation_with_slam.launch.py`: Combines Nav2 and SLAM launches.

- **maps/**: Documentation related to maps used in the project.
  - `README.md`: Instructions for creating or using maps.

- **CMakeLists.txt**: Build configuration for the package.

- **package.xml**: Metadata about the package.

## Setup Instructions

1. **Install Dependencies**: Ensure that all required dependencies for Nav2 and SLAM are installed in your ROS environment.

2. **Build the Package**: Navigate to the workspace directory and run:
   ```
   colcon build --packages-select my_nav2_slam_pkg
   ```

3. **Source the Setup File**: After building, source the setup file:
   ```
   source install/setup.bash
   ```

## Usage

To launch the navigation stack with SLAM capabilities, use the following command:
```
ros2 launch my_nav2_slam_pkg navigation_with_slam.launch.py
```

This will start the necessary nodes for navigation and SLAM, allowing your robot to navigate and map its environment simultaneously.

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests for improvements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.