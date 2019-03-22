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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

//TODO: touch sensor

@TeleOp(name = "Test: Collection and Arm", group = "Testing")
public class arm_test extends LinearOpMode {

    // Define class members
    DcMotor motor_arm;
    CRServo servo_left, servo_right;
    DigitalChannel digitalTouch;

    double left_trigger, right_trigger, power_arm;
    Boolean collecting_in, collecting_out, is_pressed;


    @Override
    public void runOpMode() {
        collecting_in = collecting_out = is_pressed = false;

        motor_arm = hardwareMap.get(DcMotor.class, "arm_drive");
        servo_left = hardwareMap.crservo.get("servo_left");
        servo_left = hardwareMap.crservo.get("servo_right");

        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the start button
        telemetry.addData(">", "Use right and left trigger to extend and retract the arm" );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            // combine the float value of both triggers
            left_trigger = 0.0 - this.gamepad1.left_trigger;
            right_trigger = this.gamepad1.right_trigger;
            power_arm = left_trigger + right_trigger;

            is_pressed = !(digitalTouch.getState());

            if(this.gamepad2.dpad_up){
                collecting_in = true;
                collecting_out = false;
            }
            else if(this.gamepad2.dpad_down){
                collecting_in = false;
                collecting_out = true;
            }
            else if(this.gamepad2.dpad_left || this.gamepad2.dpad_right){
                collecting_in = false;
                collecting_out = false;
            }

            if(collecting_in) {
                servo_left.setPower(1.0);
                servo_right.setPower(1.0);
                telemetry.addData(">", "collecting in");
            }
            else if(collecting_out){
                servo_left.setPower(-1.0);
                servo_right.setPower(-1.0);
                telemetry.addData(">", "collecting out");
            }
            else {
                servo_left.setPower(0.0);
                servo_right.setPower(0.0);
                telemetry.addData(">", "not collecting");
            }
            if(!is_pressed) {
                motor_arm.setPower(power_arm);
            }
            telemetry.addData("Arm motor Power", "%5.2f", power_arm);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();
        }

        motor_arm.setPower(0);
        telemetry.addData(">", "Done.");
        telemetry.update();

    }
}
