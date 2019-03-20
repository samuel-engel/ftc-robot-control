package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Single Joystick Drive", group = "Control")

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

            // Combining the output of both triggers to get the center motor power
            left_trigger = 0.0 - this.gamepad1.left_trigger;
            right_trigger = this.gamepad1.right_trigger;
            power_center = left_trigger + right_trigger;

            // Translate joystick coordinates to motor powers
            R_plus_L = (1.0 - Math.abs(right_x))*right_y + right_y;
            R_minus_L =(1.0 - Math.abs(right_y))*right_x + right_x;
            power_right = (R_plus_L+R_minus_L)/2;

            power_left = (R_plus_L-R_minus_L)/2;
            telemetry.addData("right", power_right*power_right*power_right );
            telemetry.addData("left", power_left*power_left*power_left);
            telemetry.update();


            //output power with easing for better control
            motor_right.setPower(Math.pow(power_right,exponent));
            motor_left.setPower(Math.pow(power_left,exponent));
            motor_center.setPower(power_center);
//          basic power output
//          motor_right.setPower(power_right);
//          motor_left.setPower(power_left);


            //ability to change the exponent with gamepad for finding optimal easing function
            telemetry.addData(">", exponent);
            if(this.gamepad1.dpad_up){
                exponent ++;
                sleep(1000);
            }
            else if(this.gamepad1.dpad_down){
                exponent --;
                sleep(1000);
            }
            telemetry.addData(">", exponent);
            telemetry.update();
        }

        telemetry.addData(">", "Done ._.");
        telemetry.update();

    }
}
