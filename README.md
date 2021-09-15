# iwr6843aop_pub
Python ROS2 pointcloud retriever for IWR6843AOPEVM mmWave device

Derived from: https://github.com/mywrong/IWR6843_TLVS

Another example: https://github.com/chopin1998/mmwave


### Prerequisites

ROS2 (Ubuntu 18.04.5 & dashing tested  // Ubuntu 20.04.3 & foxy tested)
Python3 (3.6.9 tested)

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

0. Plug in IWR6843AOPEVM, make sure ports match (default /dev/ttyUSB0,1)
1. Run ros package (make sure /opt/ros/dashing/setup.bash and ~/dev_ws/install/setup.bash are sourced)
   ```sh
   ros2 run iwr6843aop_pub pcl_pub
   ```
2. Visualize with rviz
   ```sh
   rviz2
   ```
3. 'Add' a new display (lower left corner)
4. Select 'By topic' ribbon
5. Find 'iwr6843_scan/pcl PointCloud2' and add it
6. Edit 'Fixed Frame' to 'iwr6843_frame'. (use a 'static_transform_publisher' to transform to another frame)
7. (Optional) Set point size at PointCloud2 -> Size (m) to 0.25 for better clarity

## Modify

All functional code (for the purpose of this ROS package) is located at
   ```sh
   /iwr6843aop_pub/iwr6843aop_pub/publisher_member_function.py
   ```
A number of .cfg files are provided which dictate the functionality of the mmWave device. More profiles can be made with the mmWave Demo Visualizer tool: https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.5.0/


## TODO
1. Add ROS2 parameter parsing to easily choose .cfg file and cli/data ports
2. Decrease overall latency
