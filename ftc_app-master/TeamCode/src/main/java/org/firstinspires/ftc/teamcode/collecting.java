package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(name = "Test: Collection, Arm and Pull ", group = "Testing")
public class collecting extends LinearOpMode {

    // Define class members
    DcMotor motor_arm, motor_pull;
    CRServo servo_left, servo_right;
    DigitalChannel digitalTouch;

    double left_trigger, right_trigger, power_arm;
    Boolean is_pressed = false;

    @Override
    public void runOpMode() {

        motor_arm = hardwareMap.get(DcMotor.class, "arm_drive");
        motor_pull = hardwareMap.get(DcMotor.class, "pull_drive");
        servo_left = hardwareMap.get(CRServo.class, "servo_left");
        servo_right = hardwareMap.get(CRServo.class, "servo_right");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");

        // Set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData(">", "Test the collection mechanism" );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            // Combine the float value of both triggers
            left_trigger = 0.0 - this.gamepad2.left_trigger;
            right_trigger = this.gamepad2.right_trigger;
            power_arm = left_trigger + right_trigger;

            is_pressed = !(digitalTouch.getState());

            if(this.gamepad2.dpad_up){
                servo_left.setPower(1.0);
                servo_right.setPower(1.0);
                telemetry.addData(">", "Collecting in");
            }
            else if(this.gamepad2.dpad_down){
                servo_left.setPower(-1.0);
                servo_right.setPower(-1.0);
                telemetry.addData(">", "Collecting out");
            }
            else if(this.gamepad2.dpad_left || this.gamepad2.dpad_right){
                servo_left.setPower(0.0);
                servo_right.setPower(0.0);
                telemetry.addData(">", "Stop collecting");
            }

            if(this.gamepad2.right_bumper && !is_pressed){
                motor_pull.setPower(1.0);
                telemetry.addData(">", "Landing");

            }
            else if(this.gamepad2.left_bumper){
                motor_pull.setPower(-1.0);
                telemetry.addData(">", "Pulling");

            }
            else{
                motor_pull.setPower(0.0);
            }

            motor_arm.setPower(power_arm);

            telemetry.addData("Arm motor power", "%5.2f", power_arm);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();
        }

        motor_arm.setPower(0);
        motor_pull.setPower(0);
        servo_left.setPower(0.0);
        servo_right.setPower(0.0);

        telemetry.addData(">", "Done.");
        telemetry.update();

    }
}
