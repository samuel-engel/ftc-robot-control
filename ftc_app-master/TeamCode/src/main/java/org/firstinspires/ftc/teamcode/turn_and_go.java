/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Test: Turning and Going", group = "Testing")

public class turn_and_go extends LinearOpMode
    {
    
    // The IMU sensor object
    BNO055IMU imu;

    Orientation angles;


    DcMotor motor_left, motor_right, motor_center;

    private ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {

        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");

        // Establish the initial direction of the motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        telemetry.addData(">", "IMU loaded and ready to turn" );
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData(">", angles.firstAngle);
            telemetry.update();
            turn(90, "left", 0.3);
            sleep(1000);
            turn(90, "right", 0.3);
            sleep(1000);
            turn(45, "left", 0.3);

        }
    }

    private void turn(int degree, String direction, double power) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int min_range = 0, max_range = 0;

        if(direction == "left"){
            min_range = 0;
            max_range = degree;
            motor_left.setPower(-power);
            motor_right.setPower(power);
        }
        else if(direction == "right"){
             min_range = -degree;
             max_range = 0;
             motor_left.setPower(power);
             motor_right.setPower(-power);
        }

        runtime.reset();
        while(opModeIsActive() && min_range <= angles.firstAngle && angles.firstAngle <= max_range){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "%2.5f seconds elapsed", runtime.seconds());
            telemetry.addData("Turning: ", direction);
            telemetry.addData("Current angle:", angles.firstAngle);
            telemetry.addData("> Min range: ", min_range);
            telemetry.addData("> Max range: ", max_range);
            telemetry.update();
        }
        motor_right.setPower(0);
        motor_left.setPower(0);

    }

    private void go(double power, String direction, double time) {

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
        while(opModeIsActive() && (runtime.seconds() < time)){
            telemetry.addData("Going: ", direction);
            telemetry.addData("Power: ", power);
            telemetry.addData(">", "%2.5f seconds elapsed", runtime.seconds());
            telemetry.update();
        }
        motor_right.setPower(0);
        motor_left.setPower(0);
    }

    private void push_and_reverse(double push_power, double push_time){
        //go forwards
        motor_left.setPower(push_power);
        motor_right.setPower(push_power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < push_time)) {
            telemetry.addData("Push power: ", push_power);
            telemetry.addData("Push time: ", push_time);
            telemetry.addData("Forward: ", " %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //go backwards
        motor_left.setPower(-push_power);
        motor_right.setPower(-push_power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < (push_time/2))) {
            telemetry.addData("Push power: ", push_power);
            telemetry.addData("Push time: ", push_time);
            telemetry.addData("Backward: ", " %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //stop
        motor_right.setPower(0);
        motor_left.setPower(0);
    }

}
