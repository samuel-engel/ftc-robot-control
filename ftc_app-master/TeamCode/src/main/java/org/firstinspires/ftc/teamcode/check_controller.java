package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Joystick test", group = "Gamepad")
public class check_controller extends LinearOpMode {
    // Define class members

    double right_x, right_y, left_x, left_y, R_plus_L, R_minus_L, power_right, power_left;

    @Override
    public void runOpMode() {

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            right_x = 0.0 - this.gamepad1.right_stick_x;
            right_y = 0.0 - this.gamepad1.right_stick_y;
            left_x = this.gamepad1.left_stick_x;
            left_y = this.gamepad1.left_stick_y;

            R_plus_L = (1.0 - Math.abs(right_x))*right_y + right_y;
            R_minus_L = (1.0 - Math.abs(right_y))*right_x + right_x;

            power_right = (R_plus_L+R_minus_L)/2;
            power_left = (R_plus_L-R_minus_L)/2;

            telemetry.addData("Right joystick x", right_x);
            telemetry.addData("Right joystick y", right_y);
            telemetry.addData("Right joystick x", left_x);
            telemetry.addData("Right joystick y", left_y);

            telemetry.addData("R and L", R_plus_L);
            telemetry.addData("R minus L", R_minus_L);

            telemetry.addData("Right motor", power_left);
            telemetry.addData("Left motor", power_right);

            telemetry.addData("Status", "Running");
            telemetry.update();


        }

        telemetry.addData(">", "Done ._.");
        telemetry.update();

    }
}
