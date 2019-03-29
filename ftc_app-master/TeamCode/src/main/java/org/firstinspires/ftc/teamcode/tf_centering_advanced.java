package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Webcam Proportional Centering", group = "Autonomous")

public class tf_centering_advanced extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AZVtCBn/////AAABmc0ZBKXorUd9jx9puP8gcV96rgljk0hDKL2staD0PinAX8J8c8P/UubaAwj+waF8pY3SKAGGxDh7ZfSCgr30RKxBJ/UfJkUmCbY3q8OhOGkwaSWrNk4A/BR5Sfzbw/VFRofB9n0e9jYBCJe4Rxm2kcOKa0VhER/r7VEgI2ZUhGN58BQN6ZY8j7+QUHlLTFsMm9IOAyqOc1C4QPHc5/T0tCG/mKbXOsY6l7mI6XqjUB/UmBl7I+1VhPR3hsoHJelnGqkp+uW5BqMdWwIaz7wd5D0v6Y3ZW33MjN348C32rAlwP4D4LPbI1OqSBFtS544AjbK97zojViEy1i534ykZs5MjvJYXVGiHgDxUSeMYGzOt";
    DcMotor motor_left, motor_right, motor_center;
    double power_center, left_boundary, distance_to_boundary;
    private ElapsedTime runtime = new ElapsedTime();

    int left_coordinate, screen_width, width;

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
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 1) {
                            for (Recognition recognition : updatedRecognitions) {
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


                                    // if on right half of screen (- 1 width of buffer)
                                    if (left_coordinate < left_boundary) {
                                        //the decimal percentage of distance-to-boundary of the edge-to-boundary size in pixels
                                        //left_boundary already expressed as the pixels from left edge
                                        power_center = distance_to_boundary / left_boundary;
                                        telemetry.addData("#", "Drive left");
                                        telemetry.addData("#", power_center);
                                    }
                                    //if on left half of screen (+ 1 width of buffer)
                                    else if (left_coordinate > left_boundary) {
                                        //the decimal percentage of distance-to-boundary of the edge-to-boundary size in pixels
                                        //left_boundary + width is equivalent to (screen_width + width) / 2;
                                        power_center = distance_to_boundary / (left_boundary + width);
                                        telemetry.addData("#", "Drive right");

                                    } else {
                                        motor_center.setPower(0);
                                        telemetry.addData(">", "The mineral is centered");
//                                        motor_left.setPower(0.3);
//                                        motor_right.setPower(0.3);
                                        telemetry.update();
                                    }
                                    motor_center.setPower(power_center);
                                    telemetry.addData("#", power_center);
                                }
                            }
                        } else {
                            motor_center.setPower(0);

                        }
                        telemetry.update();
                    }
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
    //Movement functions

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
        while(opModeIsActive() && (runtime.seconds() < time)){
            telemetry.addData("Going: ", direction);
            telemetry.addData("Power: ", power);
            telemetry.addData(">", "%2.5f seconds elapsed", runtime.seconds());
            telemetry.update();
        }
        motor_right.setPower(0);
        motor_left.setPower(0);
        motor_center.setPower(0);

    }

    private void push_and_reverse(double push_power, double push_time){
        // Go forwards
        motor_left.setPower(push_power);
        motor_right.setPower(push_power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < push_time)) {
            telemetry.addData("Push power: ", push_power);
            telemetry.addData("Push time: ", push_time);
            telemetry.addData("Forward: ", " %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Go backwards
        motor_left.setPower(-push_power);
        motor_right.setPower(-push_power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < (push_time/2))) {
            telemetry.addData("Push power: ", push_power);
            telemetry.addData("Push time: ", push_time);
            telemetry.addData("Backward: ", " %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop
        motor_right.setPower(0);
        motor_left.setPower(0);
    }
}