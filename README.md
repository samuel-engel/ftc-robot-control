# Java code for the Rover Ruckus robotics competition
## FIRST Tech Challenge game for the 2018-2019 season
>The object of the game is to attain a higher score than the opposing alliance by descending from the Lander, collecting Minerals from the Crater, sorting and scoring Minerals into the Cargo Hold of theLander, performing Autonomous tasks, and navigating to specific parts of the Playing Field. The Scoring Elements for the game are 60 Silver Minerals and 90 Gold Minerals,and a team supplied Team Marker.

[Rover Ruckus one page game description](https://firstinspiresst01.blob.core.windows.net/ftc/2019/gonemlpg.pdf)

## Autonomous mode

### tf_detection_webcam.java || Main Autonomous
The main autonomous mode powered by TensorFlow for recognizing the ingame minerals to detect the position of the gold mineral. The robot then uses a combination of the movement functions defined in the movement.java file to push the gold mineral from its default position and then navigate into the home depot in the corner of the playing field.

## Teleoperational modes (TeleOps)

<img src="https://user-images.githubusercontent.com/40341321/54888083-2ab5d300-4e9a-11e9-8c64-4788ff2e030c.png" width="222" title="In-app TeleOp selection">

### single_joystick_drive.java || Main Drive
The main drive translating the right joystick movements into the power of the two motors used for controlling the robot. The method includes  the mechanism established in the collecting.java files.

### tank_drive.java || Tank Drive
An alternative way of controlling the robot movements via a conventional tank drive using both joysticks each powering one side of the robot.

### collecting.java || Collection, Arm and Pull
Establishing the various collecting mechanisms including extending and lifting the collector arm, turning the collection servos and using the lifting arm.

### imu.java || IMU Sensor
Testing the inertial measurement unit (IMU) sensor to get orientation information from the gyroscope, accelerometer and geomagnetic sensor.

### webcam_basic.java || TensorFlow Webcam
Simple implementation of the TensorFlow model for recognizing the ingame minerals through a webcam and outputting the relative position of the gold mineral in relation to the silver minerals.

### movement.java || Turning; Going; Pushing
Establishing the movement functions for operating the drivetrain motors through commands for direction, time and/or power. Going, turning and pushing and reversing.

### wall_centering.java || Wall centering
A proof of concept for using the time-of-flight distance sensor to make the robot face the wall turning until the distance to the wall is minimized.

### The robot on the day of competition, 23 March 2019, in Stuttgart
<img src="https://user-images.githubusercontent.com/40341321/55036872-392bf800-501c-11e9-921a-f8e53a2fa742.jpg" width="800" title="Sofia the robot">

### Developing the Java code for our robot during one of the team sessions
<img src="https://user-images.githubusercontent.com/40341321/55037281-80ff4f00-501d-11e9-9b27-a6c645715dd9.jpg" width="600" title="One of the meetings">
