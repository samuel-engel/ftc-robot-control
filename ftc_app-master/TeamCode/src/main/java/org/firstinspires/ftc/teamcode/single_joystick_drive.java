package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Single Joystick Drive", group = "Control")

public class single_joystick_drive extends LinearOpMode {
    // Define class members
    DcMotor motor_left, motor_right, motor_center;
    double  power_left, power_right, power_center;
    double right_x, right_y, R_plus_L, R_minus_L;

    @Override
    public void runOpMode() {
        power_left = power_right = power_center = 0;
        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");
        motor_center = hardwareMap.get(DcMotor.class, "center_drive");

        // Wait for the start button
        telemetry.addData(">", "Press Start to begin driving" );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            right_x = 0.0 - this.gamepad1.right_stick_x;
            right_y = 0.0 - this.gamepad1.right_stick_y;
            power_center = this.gamepad1.left_stick_x;

            R_plus_L = (1.0 - Math.abs(right_x))*right_y + right_y;
            R_minus_L =(1.0 - Math.abs(right_y))*right_x + right_x;

            power_right = (R_plus_L+R_minus_L)/2;
            power_left = (R_plus_L-R_minus_L)/2;


            motor_right.setPower(0.0 - power_right);
            motor_left.setPower(power_left);
            motor_center.setPower(power_center);
        }

        telemetry.addData(">", "Done ._.");
        telemetry.update();

    }
}
