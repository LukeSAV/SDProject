# **SDProject**

Repo for senior design code

Link to project video: https://www.youtube.com/watch?v=O6JMgqfRybo&t=1s

## <u>Matlab</u>

Contains matlab test harness and associated files for the Extended Kalman Filter - This has since been converted to ekf_node in the ROS workspace.

## <u>SDMicro</u>

Contains all microcontroller related code for the project.

## <u>boards</u>

Contains files related to PCB design and schematics.

## <u>iOS</u>

This contains all project files related to XCode and iOS. 

##### /D2U

DeliveryInformation.swift - Singleton class used to hold relevant information to the delivery in progress.

DeliverySettingsController.swift - ViewController for the settings page. Formats and periodically updates the information on the page from FireBase.

MapMainController.swift - ViewController for the main map page. Handles map centering on desired user location and updates respective UI elements.

Waypoint.swift - Inherits MKPointAnnotation to apply color to dropped pins to distinguish between types.

## <u>ros_ws</u>

This is the ROS workspace for the project. Any packages/nodes made in ROS should be located here.

#### ROS Packages:

##### fb

This package contains the fb_node that handles interfacing to firebase. 

Nodes:

- fb_node: 
  - Publishes - /delivery_requested_status (Type: Bool)
  - Subscribes - /rtk_gpgga (Type: String)

##### rtk

This package contains the rtk_gps_node that runs code to support RTK corrections for the NS-HP-GL GPS/GNSS device. 

Nodes:

- rtk_gps_node
  - Publishes - /rtk_gpgga (Type: NavSatFix), /rtk_gpvtg (Type: HeadingSpeed)
  - Subscribes - None

##### rtk_controller

This package contains the rtk_controller node that performs path point generation to send to the microcontroller. A* algorithm, path point projection, and much more.

Nodes:

- rtk_controller_node
  - Publishes - /robot_cmd (Type: String), /next_waypoint (Type: NavSatFix), /goal_pt (Type: NavSatFix), /goal_math_pt (Type: NavSatFix)
  - Subscribes - /ekf/filtered (Type: NavSatFix), /ekf/imu/data (Type: Imu), /rtk_gpvtg (Type: String), /perception/map (Type: Image), /pitch (Type: Float32) 

##### joystick_drivers

This package contains the joy_node that interfaces the PS4 controller to ROS.

Nodes:

- joy_node: 
  - Publishes - /joy (Type: Joy)
  - Subscribes - None 

##### serial

This package contains the serial_node that handles all communication with the microcontroller.

Nodes:

- serial_node:
  - Publishes - /encoder (Type: String), /goal_string (Type: String)
  - Subscribes - /joy (Type: Joy), /delivery_requested_status (Type: Bool), /robot_cmd (Type: String)

##### perception

This package contains the lidar_node that handles creation of an obstacle map that the path planning algorithm may be run on.

Nodes:
- lidar_node:
  - Publishes - /perception/map (Type: Image)
  - Subscribes - /scan_img (Type: Image)
  
##### ekf

This package contains the ekf_node that runs the Extended Kalman Filter to fuse the robot sensor information.

Nodes:
- ekf_node:
  - Publishes - /ekf/filtered (Type: NavSatFix), /ekf/imu/data (Type: NavSatFix), /pitch (Type: Float32)
  - Subscribes - /imu/data (Type: Imu), /rtk_gpgga (Type: NavSatFix), /encoder (Type: String) 
