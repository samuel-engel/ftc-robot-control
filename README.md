## Teleoperational modes (TeleOps)

### collecting.java || Collection, Arm and Pull
Establishing the various collecting mechanisms including extending and lifting the collector arm, turning the collection servos and using the lifting arm.

### imu.java || IMU Sensor
Testing the inertial measurement unit (IMU) sensor to get orientation information from the gyroscope, accelerometer and geomagnetic sensor.

### movement.java || Turning; Going; Pushing
Establishing the movement functions for operating the drivetrain motors through commands for direction, time and/or power. Going, turning and pushing and reversing.

### single_joystick_drive.java || Main Drive
The main drive translating the right joystick movements into the power of the two motors used for controlling the robot. The method includes  the mechanism established in the collecting.java files.

### tank_drive.java || Tank Drive
An alternative way of controlling the robot movements via a conventional tank drive using both joysticks each powering one side of the robot.

### tf_detection_webcam.java || Main Autonomous
The main autonomous mode powered by TensorFlow for recognizing the ingame minerals to detect the position of the gold mineral. The robot then uses a combination of the movement functions defined in the movement.java file to push the gold mineral from its default position and then navigate into the home depot in the corner of the playing field.

### wall_centering.java || Wall centering
A proof of concept for using the time-of-flight distance sensor to make the robot face the wall turning until the distance to the wall is minimized.

### webcam_basic.java || TensorFlow Webcam
Simple implementation of the TensorFlow model for recognizing the ingame minerals through a webcam and outputting the relative position of the gold mineral in relation to the silver minerals.
