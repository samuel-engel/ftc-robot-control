/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Test: Wall centering", group = "Testing")


public class wall_centering extends LinearOpMode {

    private DistanceSensor sensorRange;

    DcMotor motor_left, motor_right;
    Boolean started_turing = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        motor_left = hardwareMap.get(DcMotor .class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");

        // Establish the initial direction of the motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);

        double new_distance, distance_cm;
        boolean wall_nearby = false;


        telemetry.addData(">>", "Press start to center on a wall");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            while(!wall_nearby){
                distance_cm = sensorRange.getDistance(DistanceUnit.CM);
                telemetry.addData("Distance in cm: ", distance_cm);
//                go(0.5, "up", 1);
                if(distance_cm < 20.0) {
                    telemetry.addData("> ", "Wall ahead");
                    telemetry.update();
                    wall_nearby = true;
                }
            }
            while(wall_nearby){
                distance_cm = sensorRange.getDistance(DistanceUnit.CM);
                telemetry.addData("Distance in cm: ", distance_cm);

                motor_right.setPower(-0.5);
                motor_left.setPower(0.5);
                runtime.reset();
                while(opModeIsActive() && (runtime.seconds() < 0.5)){
                    telemetry.addData(">", "%2.5f seconds elapsed", runtime.seconds());
                    telemetry.update();
                }
                motor_right.setPower(0);
                motor_left.setPower(0);
                new_distance = sensorRange.getDistance(DistanceUnit.CM);
                if(new_distance<distance_cm && !started_turing){
//                    turn(0.5, "right", 0.1);
                    started_turing = true;
                }
                else if (new_distance>distance_cm && !started_turing){
//                    turn(0.5, "left", 0.1);
                    started_turing = true;

                }
                else {
                    telemetry.addData("> centered ", "on wall");
                }

            }

        }
    }

}