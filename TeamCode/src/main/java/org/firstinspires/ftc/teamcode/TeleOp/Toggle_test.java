/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.Autonomous.ChampBot;


@TeleOp(name="Toggle_Test", group="ChampBot")
public class Toggle_test extends OpMode {
    ChampBot robot = new ChampBot();
    public ElapsedTime runtime = new ElapsedTime();
    boolean slowcheck = false;


    @Override
    public void init() {
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and created the configuration file.
         */
        boolean toggle = true;
        robot.init(hardwareMap);
        robot.DriveFrontLeft.setPower(0);
        robot.DriveFrontRight.setPower(0);
        robot.DriveBackLeft.setPower(0);
        robot.DriveBackRight.setPower(0);
        robot.color_sensor.enableLed(true);
    }

    public void RightStrafe(double Power) {
        robot.setDriveMotors(-Power, -Power, Power, -Power);
    }

    public void LeftStrafe(double Power) {
        robot.setDriveMotors(Power, Power, -Power, Power);
    }

    public void MotorsStopped() {
        robot.DriveFrontLeft.setPower(0);
        robot.DriveFrontRight.setPower(0);
        robot.DriveBackLeft.setPower(0);
        robot.DriveBackRight.setPower(0);
    }


    @Override
    public void loop() {
        // tank drive: each stick controls one side of the robot
        // dpad for strafing left/right

        if (gamepad1.a) {
            if (robot.toggle) {
                robot.toggle = false;
            } else if (!robot.toggle) {
                robot.toggle = true;
            }
            while (robot.toggle == true) {
                robot.DriveFrontLeft.setPower(1);
            }
            while (robot.toggle == false) {
                robot.DriveFrontLeft.setPower(0);
            }
        }

        /* Code for the arm with encoder
        if (gamepad2.x && !gamepad2.a && !gamepad2.b && !gamepad2.y && robot.cp == 1) {
            robot.zeroPos = robot.ArmMotor.getCurrentPosition();
            robot.ArmMotor.setTargetPosition(3700);
            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotor.setPower(.3);
            while (robot.ArmMotor.isBusy()) {
                telemetry.addData("Status: ", "Running to Position");
                telemetry.update();
            }
            robot.ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        }else if (gamepad2.x && !gamepad2.a && !gamepad2.b && !gamepad2.y && robot.cp == 2) {
            robot.zeroPos = robot.ArmMotor.getCurrentPosition();
            robot.ArmMotor.setTargetPosition(3100);
            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotor.setPower(.3);
            while (robot.ArmMotor.isBusy()) {
                telemetry.addData("Status: ", "Running to Position");
                telemetry.update();
            }
            robot.ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        }else if (gamepad2.x && !gamepad2.a && !gamepad2.b && !gamepad2.y && robot.cp == 3) {
            robot.ArmMotor.setTargetPosition(2900);
            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotor.setPower(.3);
            while (robot.ArmMotor.isBusy()) {
                telemetry.addData("Status: ", "Running to Position");
                telemetry.update();
            }
            robot.ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        }else if (!gamepad2.x && gamepad2.a && !gamepad2.b && !gamepad2.y) {
            robot.ArmMotor.setDirection(DcMotor.Direction.REVERSE);
            robot.ArmMotor.setTargetPosition(3700);
            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotor.setPower(.3);
            while (robot.ArmMotor.isBusy()) {
                telemetry.addData("Status: ", "Running to Position");
                telemetry.update();
            }
            robot.cp = 1;
            robot.ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        }else if (!gamepad2.x && !gamepad2.a && gamepad2.b && !gamepad2.y) {
            robot.ArmMotor.setDirection(DcMotor.Direction.REVERSE);
            robot.ArmMotor.setTargetPosition(3100);
            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotor.setPower(.3);
            while (robot.ArmMotor.isBusy()) {
                telemetry.addData("Status: ", "Running to Position");
                telemetry.update();
            }
            robot.cp = 2;
            robot.ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        }else if (!gamepad2.x && !gamepad2.a && !gamepad2.b && gamepad2.y) {
            robot.ArmMotor.setDirection(DcMotor.Direction.REVERSE);
            robot.ArmMotor.setTargetPosition(2900);
            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotor.setPower(.3);
            while (robot.ArmMotor.isBusy()) {
                telemetry.addData("Status: ", "Running to Position");
                telemetry.update();
            }
            robot.cp = 3;
            robot.ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        robot.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
    }
}

