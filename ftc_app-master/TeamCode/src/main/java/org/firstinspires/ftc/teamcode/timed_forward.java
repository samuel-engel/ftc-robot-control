package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Tank Drive", group = "Control")
public class timed_forward extends LinearOpMode {
    // Define class members
    DcMotor motor_left, motor_right;
    double  power_left, power_right;
    private ElapsedTime runtime = new ElapsedTime();
    Boolean interupt = false;


    @Override
    public void runOpMode() {
        power_left = power_right = 0;
        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");

        // Establishing the initial direction of the motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the start button
        telemetry.addData(">", "Press Start to control the robot via tank drive" );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            interupt = this.gamepad1.a;


            motor_left.setPower(0.5);
            motor_right.setPower(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0) && !interupt) {
                telemetry.addData("Path", "forward %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
//            runtime.reset();
//            motor_left.setPower(-0.5);
//            motor_right.setPower(-0.5);
//            while (opModeIsActive() && (runtime.seconds() < 1.0) && !interupt) {
//                telemetry.addData("Path", "backward %2.5f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
            motor_right.setPower(0);
            motor_left.setPower(0);
            telemetry.update();
        }

        telemetry.addData(">", "Done ._.");
        telemetry.update();

    }
}
