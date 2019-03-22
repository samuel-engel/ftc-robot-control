package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Main Drive", group = "Control")

public class single_joystick_drive extends LinearOpMode {

    // Define class members
    DcMotor motor_left, motor_right, motor_center;
    double  power_left, power_right, power_center;
    double right_x, right_y, R_plus_L, R_minus_L, left_trigger, right_trigger;
    int exponent = 1;

    @Override
    public void runOpMode() {
        power_left = power_right = power_center = 0;
        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");
        motor_center = hardwareMap.get(DcMotor.class, "center_drive");

        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);
        motor_center.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the start button
        telemetry.addData(">", "Press Start control robot via single joystick" );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            power_left = power_right = power_center = 0;
            right_x = 0.0 - this.gamepad1.right_stick_x;
            right_y = 0.0 - this.gamepad1.right_stick_y;

            // Combine the output of both triggers to get the center motor power
            left_trigger = 0.0 - this.gamepad1.left_trigger;
            right_trigger = this.gamepad1.right_trigger;
            power_center = left_trigger + right_trigger;

            // Translate joystick x and y coordinates to the power for each motor
            R_plus_L = (1.0 - Math.abs(right_x))*right_y + right_y;
            R_minus_L =(1.0 - Math.abs(right_y))*right_x + right_x;
            power_right = (R_plus_L+R_minus_L)/2;

            power_left = (R_plus_L-R_minus_L)/2;
            telemetry.addData("Power right: ", power_right );
            telemetry.addData("Power left: ", power_left);
            telemetry.addData("Power center: ", power_center);
            telemetry.update();


            motor_right.setPower(power_right);
            motor_left.setPower(power_left);
            motor_center.setPower(power_center);


        }
        motor_right.setPower(0);
        motor_left.setPower(0);
        motor_center.setPower(0);

        telemetry.addData(">", "Done driving; all motors stopped");
        telemetry.update();

    }
}
