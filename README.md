# Java code for the Rover Ruckus robotics competition
## FIRST Tech Challenge game for the 2018-2019 season
>The object of the game is to attain a higher score than the opposing alliance by descending from the Lander, collecting Minerals from the Crater, sorting and scoring Minerals into the Cargo Hold of the Lander, performing Autonomous tasks, and navigating to specific parts of the Playing Field. The Scoring Elements for the game are 60 Silver Minerals and 90 Gold Minerals,and a team supplied Team Marker.

[Rover Ruckus one page game description](https://firstinspiresst01.blob.core.windows.net/ftc/2019/gonemlpg.pdf)

## Autonomous mode
<img src="https://user-images.githubusercontent.com/40341321/55293082-76b0cc80-53f2-11e9-9ac7-5b12c41fb3da.png" width="222" title="In-app TeleOp selection" align="left">

### tf_detection_webcam.java || Main Autonomous
The main autonomous mode uses the Vuforia engine in combination with a TensorFlow model to recognize in-game minerals in order to detect the position of the gold mineral. The robot uses a combination of preprogrammed movements defined in the ```movement.java``` file to move around the field and score points. It pushes the gold mineral from its default position in the sampling area and navigates into the home depot in the corner of the playing field.

<br/><br/>
<br/><br/>


---

## Teleoperational modes (TeleOps)

<img src="https://user-images.githubusercontent.com/40341321/54888083-2ab5d300-4e9a-11e9-8c64-4788ff2e030c.png" width="222" title="In-app TeleOp selection" align="left">

### single_joystick_drive.java || Main Drive
The main drive translates the movements of the right joystick into the power of the two motors used for controlling the robot. The TeleOp also includes the collecting mechanisms established in the ```collecting.java``` file.

### tank_drive.java || Tank Drive
Tank drive is an alternative way of controlling the robot movements via a more simplistic tank drive using both joysticks where each one powers one side of the robot's drivetrain.

### collecting.java || Collection, Arm and Pull
Establishes the mechanisms for using the collecting and lifting arms. It maps extending and lifting the collector arm, turning the collection servos and using the lifting arm to the second gamepad.

### imu.java || IMU Sensor
Tests the inertial measurement unit (IMU) sensor to get orientation information from the gyroscope, accelerometer and geomagnetic sensor. 

### webcam_basic.java || TensorFlow Webcam
A simple implementation of the TensorFlow model for recognizing the in-game minerals through an external webcam and outputting the relative position of the gold mineral in relation to the silver minerals.

### movement.java || Turning; Going; Pushing
The movement functions allow giving human-like commands without powering the individual motors each time. The direction, power and/or time can be specified to automate the robot's movements.

### wall_centering.java || Wall centering
A proof of concept for using the time-of-flight distance sensor to improve the robot's orientation by facing the wall and turning until the distance to it is minimized.

The code for operational modes resides in ```ftc_app-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode```

### The robot on the day of competition, 23 March 2019, in Stuttgart
<img src="https://user-images.githubusercontent.com/40341321/55036872-392bf800-501c-11e9-921a-f8e53a2fa742.jpg" width="800" title="Side view of the robot">

### Developing the Java code for our robot during one of the team sessions
<img src="https://user-images.githubusercontent.com/40341321/55037281-80ff4f00-501d-11e9-9b27-a6c645715dd9.jpg" width="600" title="Programming during one of the meetings">
