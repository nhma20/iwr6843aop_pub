# iwr6843aop_pub

![mmw_pcl_gif](https://user-images.githubusercontent.com/76950970/194247603-18e9031a-7d34-4747-9926-9d35d6e3df4e.gif)

Python ROS2 pointcloud retriever for IWR6843AOPEVM mmWave device

Derived from: https://github.com/mywrong/IWR6843_TLVS

Another example: https://github.com/chopin1998/mmwave


### Prerequisites

- ROS2 (Ubuntu 18.04.5 & dashing tested  // Ubuntu 20.04.3 & foxy tested)
- Python3 (3.6.9 & 3.8.10 tested)
- IWR6843AOPEVM (ES2) mmWave radar device flashed with out-of-box firmware (either from this repo or inside downloaded mmwave_industrial_toolbox_x_x_x/labs/Out_Of_Box_Demo/prebuilt_binaries/ folder. Use uniflash to flash EVM (https://training.ti.com/hardware-setup-iwr6843isk-and-iwr6843isk-ods)). Set up switches as seen here:

<img src="https://user-images.githubusercontent.com/76950970/194245442-da57ecc3-3509-4173-81ec-1a4da352e732.jpg" width="600">

### Installation

1. Clone the repo to workspace
   ```sh
   cd ~/dev_ws/src/
   ```
   ```sh
   git clone https://github.com/nhma20/iwr6843aop_pub.git
   ```
2. Colcon build package
   ```sh
   cd ~/dev_ws/
   ```
   ```sh
   colcon build
   ```


<!-- USAGE EXAMPLES -->
## Usage

0. Plug in IWR6843AOPEVM, make sure CLI and data ports match (default /dev/ttyUSB0 and /dev/ttyUSB1)
1. Run ros package (make sure /opt/ros/dashing/setup.bash and <ros2_workspace>/install/setup.bash are sourced)
   ```sh
   ros2 run iwr6843aop_pub pcl_pub
   ```
   example with parameters:
   ```sh
   ros2 run iwr6843aop_pub pcl_pub /dev/ttyUSB2 /dev/ttyUSB3 /src/iwr6843isk_pub/cfg_files/xwr68xx_profile_no_grouping.cfg
   ```
2. Visualize with rviz
   ```sh
   rviz2
   ```
3. 'Add' a new display (lower left corner)
4. Select 'By topic' ribbon
5. Find 'iwr6843_scan/pcl PointCloud2' and add it
6. Edit 'Fixed Frame' to 'iwr6843_frame'. (use a 'static_transform_publisher' to transform to another frame)
7. (Optional) Set point size at PointCloud2 -> Size (m) to 0.1 for better clarity

## Modify

All functional code (for the purpose of this ROS package) is located at
   ```sh
   /iwr6843aop_pub/iwr6843aop_pub/publisher_member_function.py
   ```
A number of .cfg files are provided which dictate the functionality of the mmWave device. More profiles can be made with the mmWave Demo Visualizer tool: https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.5.0/

