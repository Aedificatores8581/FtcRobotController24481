/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="LLtest", group="Main")
public class LLtest extends LinearOpMode {

    // Declare OpMode members for each of the 4 mo tors.
    private ElapsedTime runtime = new ElapsedTime();
    private Limelight3A limelight;
    private DcMotor leftFront, rightFront, leftBack, rightBack;


    double max;
    double y;
    double x;
    double rx;

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    private Servo limelightServo;

    boolean modeIntake;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        leftBack = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBack = hardwareMap.get(DcMotor.class, "BackRight");
        limelightServo = hardwareMap.get(Servo.class, "limelightServo");

        // Reverse your appropriate motors depending on your build direction
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses START)

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            y = 0;
            x = 0;
            rx = 0;

            if ((result.isValid() && modeIntake)) {
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                y = -10 - result.getTy();
                x = 0;
                rx = -result.getTx();
            }
            if ((result.isValid() && !modeIntake)) {

            }

            frontLeftPower = y + x + rx;
            frontRightPower = y - x - rx;
            backLeftPower = y - x + rx;
            backRightPower = y + x - rx;

            mechanumDrive();

            telemetry.addData("Valid", result.isValid());
            telemetry.addData("Status", limelight.getStatus());
            telemetry.update();
        }
    }

    //Mechanum Drive code, makes it drive goods.
    public void mechanumDrive() {
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send calculated power to wheels
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);

    }
}