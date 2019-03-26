package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Test: Wall centering", group = "Testing")


public class wall_centering extends LinearOpMode {

    private DistanceSensor sensorRange;

    DcMotor motor_left, motor_right;
    Boolean started_turing = false;

    private ElapsedTime runtime = new ElapsedTime();

    //PROOF OF CONCEPT to be adapted into the autonomous phase if necessary.
    @Override
    public void runOpMode() {

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        motor_left = hardwareMap.get(DcMotor .class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");

        // Establish the initial direction of the motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);

        double new_distance, distance_cm;
        boolean wall_nearby = false;


        telemetry.addData(">>", "Press start to center on a wall");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            while(!wall_nearby){
                distance_cm = sensorRange.getDistance(DistanceUnit.CM);
                telemetry.addData("Distance in cm: ", distance_cm);
//                go(0.5, "up", 1);
                if(distance_cm < 20.0) {
                    telemetry.addData("> ", "Wall ahead");
                    telemetry.update();
                    wall_nearby = true;
                }
            }
            while(wall_nearby){
                distance_cm = sensorRange.getDistance(DistanceUnit.CM);
                telemetry.addData("Distance in cm: ", distance_cm);

                motor_right.setPower(-0.5);
                motor_left.setPower(0.5);
                runtime.reset();
                while(opModeIsActive() && (runtime.seconds() < 0.5)){
                    telemetry.addData(">", "%2.5f seconds elapsed", runtime.seconds());
                    telemetry.update();
                }
                motor_right.setPower(0);
                motor_left.setPower(0);
                new_distance = sensorRange.getDistance(DistanceUnit.CM);
                if(new_distance<distance_cm && !started_turing){
//                    turn(0.5, "right", 0.1);
                    started_turing = true;
                }
                else if (new_distance>distance_cm && !started_turing){
//                    turn(0.5, "left", 0.1);
                    started_turing = true;

                }
                else {
                    telemetry.addData("> centered ", "on wall");
                }

            }

        }
    }

}