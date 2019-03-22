/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//TODO: Look around
//TODO: A copy for the second possibility; last step
//TODO: Find a way to change parameters conveniently while running; IMPORTANT
//TODO: Descending down with the arm ; easy look at collecting

@Autonomous(name = "Main Autonomous", group = "Autonomous")

public class tf_detection_webcam extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AZVtCBn/////AAABmc0ZBKXorUd9jx9puP8gcV96rgljk0hDKL2staD0PinAX8J8c8P/UubaAwj+waF8pY3SKAGGxDh7ZfSCgr30RKxBJ/UfJkUmCbY3q8OhOGkwaSWrNk4A/BR5Sfzbw/VFRofB9n0e9jYBCJe4Rxm2kcOKa0VhER/r7VEgI2ZUhGN58BQN6ZY8j7+QUHlLTFsMm9IOAyqOc1C4QPHc5/T0tCG/mKbXOsY6l7mI6XqjUB/UmBl7I+1VhPR3hsoHJelnGqkp+uW5BqMdWwIaz7wd5D0v6Y3ZW33MjN348C32rAlwP4D4LPbI1OqSBFtS544AjbK97zojViEy1i534ykZs5MjvJYXVGiHgDxUSeMYGzOt";
    private ElapsedTime runtime = new ElapsedTime();
    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;

    double  power_center = 0, push_time, push_power, side_time, side_power;
    DcMotor motor_left, motor_right, motor_center;
    int left_coordinate, screen_width, width, left_boundary, distance_to_boundary;
    String gold_mineral_position = null;
    Boolean is_gold_ahead;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        motor_center = hardwareMap.get(DcMotor.class, "center_drive");
        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");
        // Establishing the initial direction of the motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        push_time = 3.0;
        push_power = 0.5;
        side_time = 2.0;
        side_power = 0.5;

        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
//          right negative ; left positive
            while (opModeIsActive()) {
                if (tfod != null && gold_mineral_position == null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    //if detecting something
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());


                        // if three objects detected, find the position of the golden mineral
                        if (updatedRecognitions.size() == 3 && gold_mineral_position == null) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                screen_width = recognition.getImageWidth();
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    gold_mineral_position = "left";

                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    gold_mineral_position = "right";

                                } else {
                                    gold_mineral_position = "center";

                                }
                            }
                        }
                        telemetry.update();
                    }
                    else {
                        telemetry.addData(">", "Nothing detected");
                        telemetry.update();
                    }
                }
                if(gold_mineral_position.equals("left")){
                    go(side_power, "left", side_time);
                    push_and_reverse(push_power, push_time);
                    go_to_depot();
                }
                else if(gold_mineral_position.equals("right")){
                    go(side_power, "right", side_time);
                    push_and_reverse(push_power, push_time);
                    go_to_depot();
                }
                else{
                    push_and_reverse(push_power, push_time);
                    go_to_depot();
                }




            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }



    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void go_to_depot(){
        go(0.5, "left", 2);
        turn(45,"left", 0.3);
        go(0.5, "up", 3);
        go(0.5, "left", 5); // or right depends on the side

    }

    //Movement functions


    //OLD
    private void go(double power, String direction, double time) {

        switch(direction) {
            case "left":
                motor_center.setPower(power);
                break;
            case "right":
                motor_center.setPower(-power);
                break;
            case "up":
                motor_left.setPower(power);
                motor_right.setPower(power);
                break;
            case "down":
                motor_left.setPower(-power);
                motor_right.setPower(-power);
                break;
            default:
                motor_left.setPower(0);
                motor_right.setPower(0);
                motor_center.setPower(0);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData(">", " %2.5f S Elapsed", runtime.seconds());
            telemetry.addData(">", direction);
            telemetry.update();
        }
        motor_right.setPower(0);
        motor_left.setPower(0);

    }
    //OLD
    private void push_and_reverse(double push_power, double push_time){
        telemetry.addData(">", push_power);
        telemetry.addData(">", push_time);

        //go forwards
        motor_left.setPower(push_power);
        motor_right.setPower(push_power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < push_time)) {

        }

        //go backwards
        motor_left.setPower(-push_power);
        motor_right.setPower(-push_power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < (push_time))) {

        }
        //stop
        motor_right.setPower(0);
        motor_left.setPower(0);
        telemetry.update();
    }
    //OLD
    private void turn(int degree, String direction, double power) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int min_range;
        int max_range;
        if(direction == "left"){
            min_range = 0;
            max_range = degree;
            motor_left.setPower(-power);
            motor_right.setPower(power);
        }
        else if(direction == "right"){
            min_range = -degree;
            max_range = 0;
            motor_left.setPower(power);
            motor_right.setPower(-power);
        }
        else {
            min_range = 0;
            max_range = 0;
            motor_left.setPower(0.0);
            motor_right.setPower(0.0);
            telemetry.addData(">", "Incorrect input: use either left or right");
            telemetry.update();
        }
        telemetry.addData("> min - ", min_range);
        telemetry.addData("> max - ", max_range);
        telemetry.update();
        runtime.reset();
        while(opModeIsActive() && min_range <= angles.firstAngle && angles.firstAngle <= max_range){
            telemetry.addData(">", "%2.5f seconds elapsed", runtime.seconds());
            telemetry.addData("> turning ", direction);
            telemetry.addData("> current angle - ", angles.firstAngle);
            telemetry.update();
        }
        motor_right.setPower(0);
        motor_left.setPower(0);

    }
}

