package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Timed forward", group = "Control")
public class timed_forward extends LinearOpMode {
    // Define class members
    DcMotor motor_left, motor_right;
    double  power;
    private ElapsedTime runtime = new ElapsedTime();
    Boolean interupt = false;


    @Override
    public void runOpMode() {
        power = 0.3;
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

            motor_left.setPower(power);
            motor_right.setPower(power);
            telemetry.addData(">", runtime.seconds());
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0) && !interupt) {
                interupt = this.gamepad1.a;
                telemetry.addData(">", "forward %2.5f S Elapsed", runtime.seconds());
                telemetry.addData(">", interupt);
                telemetry.addData(">", runtime);
                telemetry.update();
            }

//            Backwards
//            motor_left.setPower(-power);
//            motor_right.setPower(-power);
//            runtime.reset();
//            while (opModeIsActive() && (runtime.seconds() < 2.0) && !interupt) {
//                interupt = this.gamepad1.a;
//                telemetry.addData("Path", "backward %2.5f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }

            // turn
            motor_left.setPower(power);
            motor_right.setPower(-power);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0) && !interupt) {
                telemetry.addData("Path", "sideways %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            motor_right.setPower(0);
            motor_left.setPower(0);
            telemetry.update();
            sleep(5000);
        }

        telemetry.addData(">", "Done ._.");
        telemetry.update();

    }
}
