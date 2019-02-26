# **SDProject**

Repo for senior design app code



## <u>BluetoothTuner</u>



## <u>GroundDetector</u>



## <u>Robot</u>



## <u>Sabertooth_Driver</u>

## Modified example of interfacing with the Sabertooth motor controller.

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
  - Publishes - None
  - Subscribes - rtk_gpgga (Type: String)

##### rtk

This package contains the rtk_gps_node that runs code to support RTK corrections for the NS-HP-GL GPS/GNSS device. 

Nodes:

- rtk_gps_node
  - Publishes - rtk_gpgga (Type: String)
  - Subscribes - None

##### rtk_controller

Nodes:

- rtk_controller_node
  - Publishes - None
  - Subscribes - rtk_gpgga(Type: String)

##### joystick_drivers

This package contains the joy_node that interfaces the PS4 controller to ROS.

Nodes:

- joy_node: 
  - Publishes - joy (Type: Joy)
    - Subscribes - None 

##### rc

This package contains the rc_listener node that listens for RC inputs and converts that to requests sent to the microcontroller

Nodes:

- rc_listener:
  - Publishes - None
  - Subscribes - joy (Type: Joy)

##### serial

This package will interface with the UART pins on the Jetson. 

Nodes:

- serial_node:
  - Publishes - None
  - Subscribes - packet (Type: String)

##### path_planning

This package contains the path_planning_node that uses the adjusted robot position and map provided by the perception layer to plan a local path that targets global waypoints.

Nodes:

- path_planning_node:
  - Publishes - packet (Type: String)
  - Subscribes - TBD