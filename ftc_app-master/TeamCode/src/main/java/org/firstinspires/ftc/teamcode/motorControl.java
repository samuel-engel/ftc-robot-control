package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Control motor via left joystick", group = "Motors")

public class motorControl extends LinearOpMode {

    // Define class members
    DcMotor motor;
    double  power = 0;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "left_drive");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {
            power = -this.gamepad1.left_stick_y;
            motor.setPower(power);
            telemetry.addData("Target Power", power);
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }

        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
