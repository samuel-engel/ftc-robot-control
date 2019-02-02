package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Check all motors", group = "Motors")

public class motorsCheck extends LinearOpMode {

    // Define class members
    DcMotor motor_left, motor_right, motor_center;
    double  power   = 0;
    boolean ramp_up  = true;


    @Override
    public void runOpMode() {

        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "center_drive");
        motor_center = hardwareMap.get(DcMotor.class, "right_drive");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
            power += 0.2;

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the motor to the new power and pause;
            motor_left.setPower(power);
            sleep(100);
            idle();
        }

        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
