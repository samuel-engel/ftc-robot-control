package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tank Drive", group = "Control")
public class tank_drive extends LinearOpMode {
    // Define class members
    DcMotor motor_left, motor_right, motor_center;
    double  power_left, power_right, power_center;
    double  left_trigger, right_trigger;


    @Override
    public void runOpMode() {
        power_left = power_right = power_center = 0;
        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");
        motor_center = hardwareMap.get(DcMotor.class, "center_drive");

        // Establishing the initial direction of the motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the start button
        telemetry.addData(">", "Press Start to control the robot via tank drive" );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Reversing the y axis to make joystick up = 1 and vica verca
            power_right = 0.0 - this.gamepad1.right_stick_y;
            power_left = 0.0 - this.gamepad1.left_stick_y;

            // Combining the output of both triggers to get the center motor power
            left_trigger = 0.0 - this.gamepad1.left_trigger;
            right_trigger = this.gamepad1.right_trigger;
            power_center = left_trigger + right_trigger;

            telemetry.addData("Right motor power", power_right);
            telemetry.addData("Left motor power", power_left);
            telemetry.addData("Center motor power", power_center);
            telemetry.update();

            motor_right.setPower(power_right);
            motor_left.setPower(power_left);
            motor_center.setPower(power_center);
        }

        telemetry.addData(">", "Done ._.");
        telemetry.update();

    }
}
