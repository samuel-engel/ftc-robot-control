package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Push mineral", group = "Control")
public class push_mineral extends LinearOpMode {

    DcMotor motor_left, motor_right, motor_center;
    double  push_power, push_time;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        //set initial push power and duration
        push_power = 0.3;
        push_time = 1.0;

        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");

        // establish the initial direction of the motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the start button
        telemetry.addData(">", "Press Start to push robot and return" );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //adjust parameters via gamepad for calibration
            if(this.gamepad1.dpad_up){
                push_power += 0.1;
                sleep(1000);
            }
            else if (this.gamepad1.dpad_down){
                push_power -= 0.1;
                sleep(1000);
            }
            else if (this.gamepad1.dpad_right){
                push_time += 1.0;
                sleep(1000);

            }
            else if (this.gamepad1.dpad_left) {
                push_time -= 1.0;
                sleep(1000);

            }
            telemetry.addData(">", push_power);
            telemetry.addData(">", push_time);
            telemetry.update();


            //go forwards
            motor_left.setPower(push_power);
            motor_right.setPower(push_power);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < push_time)) {
                telemetry.addData(">", "forward %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            //go backwards
            motor_left.setPower(-push_power);
            motor_right.setPower(-push_power);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < (push_time/2))) {
                telemetry.addData("Path", "backwards %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            //stop
            motor_right.setPower(0);
            motor_left.setPower(0);
            telemetry.update();
            sleep(5000);
        }

        telemetry.addData(">", "Done ._.");
        telemetry.update();

    }

    private void go(double power, String direction, int time) {

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
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData(">", " %2.5f S Elapsed", runtime.seconds());
            telemetry.addData(">", direction);
            telemetry.update();
        }
        motor_right.setPower(0);
        motor_left.setPower(0);

    }
}
