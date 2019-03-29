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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "webcam centering advanced", group = "Testing")
//@Disabled
public class tf_centering_advanced extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AZVtCBn/////AAABmc0ZBKXorUd9jx9puP8gcV96rgljk0hDKL2staD0PinAX8J8c8P/UubaAwj+waF8pY3SKAGGxDh7ZfSCgr30RKxBJ/UfJkUmCbY3q8OhOGkwaSWrNk4A/BR5Sfzbw/VFRofB9n0e9jYBCJe4Rxm2kcOKa0VhER/r7VEgI2ZUhGN58BQN6ZY8j7+QUHlLTFsMm9IOAyqOc1C4QPHc5/T0tCG/mKbXOsY6l7mI6XqjUB/UmBl7I+1VhPR3hsoHJelnGqkp+uW5BqMdWwIaz7wd5D0v6Y3ZW33MjN348C32rAlwP4D4LPbI1OqSBFtS544AjbK97zojViEy1i534ykZs5MjvJYXVGiHgDxUSeMYGzOt";
    double  power_center;
    DcMotor motor_center;

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

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
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

                        //for every recognized object
                        for (Recognition recognition : updatedRecognitions) {
                            //if one of the recognized objects is the gold mineral
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                left_coordinate = (int) recognition.getLeft();
                                screen_width = recognition.getImageWidth();
                                width = (int) recognition.getWidth();
                                // define left boundary at half the screen size - half the width of the object
                                // if left coordinate equals boundary, object is horizontally centered
                                left_boundary = (screen_width - width) / 2;

                                telemetry.addData("# Left", left_coordinate);
                                telemetry.addData("# Screen width", screen_width);
                                telemetry.addData("# Width", width);
                                telemetry.addData("# Boundary", left_boundary);

                                //difference between the left coordinate and boundary in pixels
                                distance_to_boundary = (left_boundary - left_coordinate);


                                // if on right half of screen
                                if (left_coordinate < left_boundary) {
                                    //the decimal percentage of distance-to-boundary of the edge-to-boundary size in pixels
                                    //left_boundary already expressed as the pixels from left edge
                                    power_center = (double) distance_to_boundary / left_boundary;
                                    telemetry.addData("#", "Drive left");
                                    telemetry.addData("#", power_center);
                                }
                                //if on left half of screen
                                else if (left_coordinate > left_boundary) {
                                    //the decimal percentage of distance-to-boundary of the edge-to-boundary size in pixels
                                    //left_boundary + width is equivalent to (screen_width + width) / 2;
                                    power_center = (double) distance_to_boundary / (left_boundary + width);
                                    telemetry.addData("#", "Drive right");

                                }
                                //in middle
                                //TODO: increase boundary to leave +- width to count as 'in center'
                                else {
                                    motor_center.setPower(0);
                                    is_gold_ahead = true;
                                    tfod.shutdown();
                                }
                                motor_center.setPower(power_center);
                                telemetry.addData("#", power_center);

                            }
                            else {
                                //look around if nothing happens?
                                motor_center.setPower(0);

                            }
                        }
                        telemetry.update();
                    }
                }
                if(is_gold_ahead) {
                    //go forward
                    //TODO: time based driving
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
}
