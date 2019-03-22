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

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link turn_45} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "Turn 45", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class turn_45 extends LinearOpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    Position position;
    Velocity velocity;

    DcMotor motor_left, motor_right, motor_center;
    double  power;
    private ElapsedTime runtime = new ElapsedTime();

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        motor_left = hardwareMap.get(DcMotor.class, "left_drive");
        motor_right = hardwareMap.get(DcMotor.class, "right_drive");

        // Establishing the initial direction of the motors
        motor_right.setDirection(DcMotor.Direction.REVERSE);
        motor_left.setDirection(DcMotor.Direction.FORWARD);



        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait for the start button
        telemetry.addData(">", "Loaded" );
        telemetry.update();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            // turn
            //left positive right negative

            telemetry.addData(">", angles.firstAngle);
            telemetry.update();
            turn(90, "left", 0.3);
            turn(90, "right", 0.3);
            sleep(10000);
            go(0.5, "up", 2.0);

        }
    }


    private void turn(int degree, String direction, double power) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int min_range;
        int max_range;
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
        else {
            min_range = 0;
            max_range = 0;
            motor_left.setPower(0.0);
            motor_right.setPower(0.0);
            telemetry.addData(">", "Incorrect input: use either left or right");
            telemetry.update();
        }
        telemetry.addData("> min - ", min_range);
        telemetry.addData("> max - ", max_range);
        telemetry.update();
        runtime.reset();
        while(opModeIsActive() && min_range <= angles.firstAngle && angles.firstAngle <= max_range){
            telemetry.addData(">", "%2.5f seconds elapsed", runtime.seconds());
            telemetry.addData("> turning ", direction);
            telemetry.addData("> current angle - ", angles.firstAngle);
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

        telemetry.addData("> direction - ", direction);
        telemetry.update();
        runtime.reset();
        //figure out a way to capture distance
        while(opModeIsActive() && (runtime.seconds() < time)){
            telemetry.addData(">", "%2.5f seconds elapsed", runtime.seconds());
            telemetry.addData("> going ", direction);
            telemetry.addData("> power ", power);
            telemetry.update();
        }
        motor_right.setPower(0);
        motor_left.setPower(0);

    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.

                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                position = imu.getPosition();
                velocity = imu.getVelocity();
                }
            });

        /*telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
            .addData("grvty", new Func<String>() {
                @Override public String value() {
                    return gravity.toString();
                    }
                })
            .addData("mag", new Func<String>() {
                @Override public String value() {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel
                                    + gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));
                    }
                });*/
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
