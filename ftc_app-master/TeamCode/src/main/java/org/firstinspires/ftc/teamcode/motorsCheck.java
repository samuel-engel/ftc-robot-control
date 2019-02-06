package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Check all motors", group = "Motors")

public class motorsCheck extends LinearOpMode {

    // Define class members
    DcMotor motor_left, motor_right, motor_center;
    double  power   = 0;
    int i = 0;
    @Override
    public void runOpMode() {

        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");
        motor_center = hardwareMap.get(DcMotor.class, "center_drive");
        DcMotor[] motors = {motor_left, motor_right, motor_center};

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
            power += 0.2;

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData("Motor Name", "%s", motors[i]);
            telemetry.update();

            motors[i].setPower(power);

            if(power >= 1.0) {
                motors[i].setPower(0);
                i++;
            }
            sleep(100);
            idle();
            if(i>2) break;
        }

        telemetry.addData(">", "Done checking");
        telemetry.update();

    }
}
