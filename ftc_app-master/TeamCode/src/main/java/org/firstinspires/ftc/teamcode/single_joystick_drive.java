package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Main Drive", group = "Control")

public class single_joystick_drive extends LinearOpMode {

    // Define class members
    DcMotor motor_left, motor_right, motor_center, motor_arm, motor_pull;
    CRServo servo_left, servo_right;
    double  power_left, power_right, power_center, power_arm;
    double right_x, right_y, R_plus_L, R_minus_L, left_trigger, right_trigger;

    @Override
    public void runOpMode() {
        power_left = power_right = power_center = 0;
        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");
        motor_center = hardwareMap.get(DcMotor.class, "center_drive");
        motor_arm = hardwareMap.get(DcMotor.class, "arm_drive");
        motor_pull = hardwareMap.get(DcMotor.class, "pull_drive");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        servo_right = hardwareMap.get(CRServo.class, "servo_right");


        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.REVERSE);
        motor_center.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the start button
        telemetry.addData(">", "Press Start control robot via single joystick" );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            power_left = power_right = power_center = 0;
            right_x = 0.0 - this.gamepad1.right_stick_x;
            right_y = 0.0 - this.gamepad1.right_stick_y;

            // combine the float value of both triggers SECOND
            left_trigger = 0.0 - this.gamepad2.left_trigger;
            right_trigger = this.gamepad2.right_trigger;
            power_arm = left_trigger + right_trigger;
            telemetry.addData(">", left_trigger);
            telemetry.addData(">", right_trigger);
            telemetry.addData(">", power_arm);

            // Combine the output of both triggers to get the center motor power
            left_trigger = 0.0 - this.gamepad1.left_trigger;
            right_trigger = this.gamepad1.right_trigger;
            power_center = left_trigger + right_trigger;

            if(this.gamepad2.right_bumper){
                motor_pull.setPower(1.0);
            }
            else if(this.gamepad2.left_bumper){
                motor_pull.setPower(-1.0);
            }
            else{
                motor_pull.setPower(0.0);
            }

            motor_arm.setPower(power_arm);

            if(this.gamepad2.dpad_up){
                servo_left.setPower(1.0);
                servo_right.setPower(1.0);
                telemetry.addData(">", "collecting in");
            }
            else if(this.gamepad2.dpad_down){
                servo_left.setPower(-1.0);
                servo_right.setPower(-1.0);
                telemetry.addData(">", "collecting out");
            }
            else if(this.gamepad2.dpad_left || this.gamepad2.dpad_right){
                servo_left.setPower(0.0);
                servo_right.setPower(0.0);
                telemetry.addData(">", "not collecting");
            }

            // Translate joystick x and y coordinates to the power for each motor
            R_plus_L = (1.0 - Math.abs(right_x))*right_y + right_y;
            R_minus_L =(1.0 - Math.abs(right_y))*right_x + right_x;
            power_right = (R_plus_L+R_minus_L)/2;

            power_left = (R_plus_L-R_minus_L)/2;
            telemetry.addData("Power right: ", power_right );
            telemetry.addData("Power left: ", power_left);
            telemetry.addData("Power center: ", power_center);
            telemetry.update();

            /*if (this.gamepad2.right_bumper) {
                motor_pull.setPower(1.0);
            } else if (this.gamepad2.left_bumper) {
                motor_pull.setPower(-1.0);
            } else {
                motor_pull.setPower(0.0);
            }*/


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
