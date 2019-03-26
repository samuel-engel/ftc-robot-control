package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Test: Turning; Going; Pushing", group = "Testing")

public class movement extends LinearOpMode
    {

    // The IMU sensor object
    BNO055IMU imu;

    Orientation angles;
    DcMotor motor_left, motor_right, motor_center;

    private ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {

        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");
        motor_center = hardwareMap.get(DcMotor.class, "center_drive");

        // Establish the initial direction of the motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        telemetry.addData(">", "IMU loaded and ready to turn" );
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            go(0.5, "left", 1);
            sleep(1000);
            push_and_reverse(0.5, 2);
            sleep(1000);

        }
    }

    private void turn(int degree, String direction, double power) {
        // Angles from 0 to -180 for turing left and 0 to 180 for turning right
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int min_range = 0, max_range = 0;

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

        runtime.reset();
        while(opModeIsActive() && min_range <= angles.firstAngle && angles.firstAngle <= max_range){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "%2.5f seconds elapsed", runtime.seconds());
            telemetry.addData("Turning: ", direction);
            telemetry.addData("Current angle:", angles.firstAngle);
            telemetry.addData("> Min range: ", min_range);
            telemetry.addData("> Max range: ", max_range);
            telemetry.update();
        }
        motor_right.setPower(0);
        motor_left.setPower(0);

    }

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
