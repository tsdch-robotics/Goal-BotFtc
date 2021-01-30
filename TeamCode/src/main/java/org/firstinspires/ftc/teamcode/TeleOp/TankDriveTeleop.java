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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.ChampBot;


@TeleOp(name="Tank Drive", group="ChampBot")
public class TankDriveTeleop extends OpMode {
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
    }


    @Override
    public void loop() {
        // tank drive: each stick controls one side of the robot
        // dpad for strafing left/right
        float DriveLeftPower = gamepad1.left_stick_y;
        float DriveRightPower = gamepad1.right_stick_y;
        boolean LeftStrafe = gamepad1.dpad_left;
        boolean RightStrafe = gamepad1.dpad_right;
        boolean WheelToggle = false;

        if (RightStrafe) {
            // to right strafe, right motors towards each other, left motors away from each other
            robot.DriveFrontLeft.setPower(-1);
            robot.DriveFrontRight.setPower(1);
            robot.DriveBackLeft.setPower(1);
            robot.DriveBackRight.setPower(-1);
        } else if (LeftStrafe) {
            // opposite of right strafe
            robot.DriveFrontLeft.setPower(1);
            robot.DriveFrontRight.setPower(-1);
            robot.DriveBackLeft.setPower(-1);
            robot.DriveBackRight.setPower(1);
        }
        if (gamepad1.y) {
            robot.WheelMotor.setPower(-.75);
            robot.IntakeMotor.setPower(1);
        } else {
            robot.WheelMotor.setPower(0);
            robot.ArmMotor.setPower(0);
            robot.IntakeMotor.setPower(0);
        }
        if (gamepad1.left_bumper) {
            robot.WheelMotor.setPower(.75);
            robot.IntakeMotor.setPower(-1);
        } else {
            robot.WheelMotor.setPower(0);
            robot.IntakeMotor.setPower(0);
        }
        if (gamepad1.dpad_up) {
            robot.ArmMotor.setPower(1);
        } else {
            robot.WheelMotor.setPower(0);
            robot.ArmMotor.setPower(0);
        }
        if (gamepad1.dpad_down) {
            robot.ArmMotor.setPower(-1);
        } else {
            robot.WheelMotor.setPower(0);
            robot.ArmMotor.setPower(0);
        }
        if (gamepad1.b) {
            robot.WheelMotor.setPower(0);
            robot.ArmMotor.setPower(0);
        } else {
            robot.WheelMotor.setPower(0);
            robot.ArmMotor.setPower(0);
        }
        if (gamepad1.right_bumper) {
            robot.LauncherMotor.setPower(25);
        } else {
                robot.LauncherMotor.setPower(0);
            }

            robot.setDriveMotors(DriveLeftPower, DriveRightPower, DriveLeftPower, DriveRightPower);
        }
    }

