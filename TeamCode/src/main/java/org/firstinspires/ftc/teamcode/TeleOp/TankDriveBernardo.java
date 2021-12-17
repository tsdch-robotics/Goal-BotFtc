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


@TeleOp(name="TankDriveBernardo", group="ChampBot")
public class TankDriveBernardo extends OpMode {
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

        telemetry.addData("Red: ", robot.color_sensor.red());
        telemetry.addData("Green: ", robot.color_sensor.green());
        telemetry.addData("Blue: ", robot.color_sensor.blue());

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double DLY = gamepad1.left_stick_y * .75;
        double DRY = gamepad1.right_stick_y * .75;
        float DLX = Math.abs(gamepad1.left_stick_x);
        float DRX = Math.abs(gamepad1.right_stick_x);
        //float rightPower = DLY / DLX;
        //float leftPower = DRY / DRX;
        double inf = Double.POSITIVE_INFINITY;

        telemetry.addData("Right: ", DRY);
        telemetry.addData("Left: ", DLY);
        /*if (leftPower == inf && rightPower == inf) {
            leftPower = 1;
            rightPower = 1;
        } else if (leftPower == -inf && rightPower == -inf) {
            leftPower = -1;
            rightPower = -1;
        } else if (leftPower == inf) {
            leftPower = 1;
        } else if (leftPower == -inf) {
            leftPower = -1;
        } else if (rightPower == inf) {
            rightPower = 1;
        } else if (rightPower == -inf) {
            rightPower = -1;
        } else if (leftPower > 1) {
            leftPower = 1;
        } else if (rightPower > 1) {
            rightPower = 1;
        }
            leftPower = (float) (leftPower * 0.75);
            rightPower = (float) (rightPower  *0.75);
            telemetry.addData("Power R: ", rightPower);
            telemetry.addData("Power L: ", leftPower);
*/

        if (gamepad1.dpad_right){
            robot.setDriveMotors(.75, -.75, -.75, .75);
        }else if (gamepad1.dpad_left) {
            robot.setDriveMotors(-.75, .75, .75, -.75);
        }else if (gamepad1.dpad_up){
            robot.setDriveMotors(.75, .75, .75, .75);
        }else if (gamepad1.dpad_down) {
            robot.setDriveMotors(-.75, -.75, -.75, -.75);
        }

        if (gamepad1.right_trigger > 0) {
            robot.ArmMotor.setPower(.5);
        } else if (gamepad1.left_trigger > 0) {
            robot.ArmMotor.setPower(-.5);
        } else {
            robot.ArmMotor.setPower(0);
        }

        if (gamepad1.x) {
            robot.CarouselMotor1.setPower(.5);
        }else if (gamepad1.b) {
            robot.CarouselMotor2.setPower(-.5);
        }else {
            robot.CarouselMotor1.setPower(0);
            robot.CarouselMotor2.setPower(0);
        }
        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            robot.Claw.setPosition(1);
        }else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            robot.Claw.setPosition(0);
        }
        if (!gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            robot.setDriveMotors(-DLY, -DRY, -DLY, -DRY);
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
