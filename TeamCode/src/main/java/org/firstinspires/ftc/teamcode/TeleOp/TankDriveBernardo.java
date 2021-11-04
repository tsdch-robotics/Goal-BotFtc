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
import java.lang.*;

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
    }

    public void RightStrafe(double Power) {
        robot.DriveFrontLeft.setPower(-Power);
        robot.DriveFrontRight.setPower(-Power);
        robot.DriveBackLeft.setPower(-Power);
        robot.DriveBackRight.setPower(-Power);
    }

    public void RightStrafeSlow(double Power) {
        robot.DriveFrontLeft.setPower(-Power/2);
        robot.DriveFrontRight.setPower(-Power/2);
        robot.DriveBackLeft.setPower(-Power/2);
        robot.DriveBackRight.setPower(-Power/2);
    }

    public void LeftStrafe(double Power) {
        robot.DriveFrontLeft.setPower(Power);
        robot.DriveFrontRight.setPower(Power);
        robot.DriveBackLeft.setPower(Power);
        robot.DriveBackRight.setPower(Power);
    }

    public void LeftStrafeSlow(double Power) {
        robot.DriveFrontLeft.setPower(Power);
        robot.DriveFrontRight.setPower(Power);
        robot.DriveBackLeft.setPower(Power);
        robot.DriveBackRight.setPower(Power);
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
        float DLY = gamepad1.left_stick_y;
        float DRY = gamepad1.right_stick_y;
        float DLX = Math.abs(gamepad1.left_stick_x);
        float DRX = Math.abs(gamepad1.right_stick_x);
        float rightPower = DLY / DLX;
        float leftPower = DRY / DRX;
        float rightPowerSlow = rightPower / 2;
        float leftPowerSlow = leftPower / 2;
        double inf = Double.POSITIVE_INFINITY;

        telemetry.addData("L: ", leftPower);
        telemetry.addData("R: ", rightPower);
        telemetry.addData("Slow L: ", leftPowerSlow);
        telemetry.addData("Slow R: ", rightPowerSlow);
        if (gamepad1.left_bumper) {
            if (leftPower == inf && rightPower == inf) {
                rightPower = 1;
                leftPower = 1;
            }else if (leftPower == -inf && rightPower == -inf){
                rightPower = -1;
                leftPower = -1;
            }
            leftPower = leftPower/2;
            rightPower =rightPower/2;
            telemetry.addData("Power R: ", rightPower);
            telemetry.addData("Power L: ", leftPower);
            if (gamepad1.dpad_right) {
                LeftStrafeSlow(.5);
            } else if (gamepad1.dpad_left) {
                RightStrafeSlow(.5);
            }
            robot.setDriveMotors(-rightPower, leftPower, -rightPower, -leftPower);
        }else if (gamepad1.left_bumper == false) {
            robot.setDriveMotors(-rightPower, leftPower, -rightPower, -leftPower);
            if (gamepad1.dpad_right){
                LeftStrafe(1);
            }else if (gamepad1.dpad_left) {
                RightStrafe(1);
            }

        }


        /*if (robot.toggle = true && gamepad1.x) {  // Only execute once per Button push
            robot.toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
        }else if (robot.toggle) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
            while (robot.DMT = false) {
                if (gamepad1.x) {
                    robot.DMT = true;
                } else if (gamepad1.dpad_right) {
                    LeftStrafe();
                } else if (gamepad1.dpad_left) {
                    RightStrafe();
                } else
                    robot.setDriveMotors(-rightPower, leftPower, -rightPower, -leftPower);
            }
            robot.toggle = true;
        }else if (robot.toggle = false) {
            while (robot.DMT) {
                if (gamepad1.x){
                    robot.DMT= false;
                } else if (gamepad1.dpad_right) {
                    // opposite of right strafe
                    LeftStrafeSlow();
                }else if (gamepad1.dpad_left){
                    RightStrafeSlow();
                }
                else
                    robot.setDriveMotors(-rightPower/2, leftPower/2, -rightPower/2, -leftPower/2);
                }
            robot.toggle = true;
        }else if(gamepad1.x == false) {
            robot.toggle = true; // Button has been released, so this allows a re-press to activate the code above.
        }

        /*if (gamepad1.y) {
            robot.WheelMotor.setPower(1);
            //robot.IntakeMotor.setPower(1);
        } else {
            robot.WheelMotor.setPower(0);
            robot.ArmMotor.setPower(0);
            //robot.IntakeMotor.setPower(0);
        }
        if (gamepad1.left_bumper) {
            robot.WheelMotor.setPower(-1);
            //robot.IntakeMotor.setPower(-1);
        } else {
            robot.WheelMotor.setPower(0);
            //robot.IntakeMotor.setPower(0);
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
            //if (gamepad1.right_bumper) {
            //robot.LauncherMotor.setPower(-1);
            //}
            //else {
            // robot.LauncherMotor.setPower(0);
            //}


            //if (gamepad1.left_bumper) {
                //DriveLeftPower = DriveLeftPower / 2;
                //DriveRightPower = DriveRightPower / 2;
            //}

            robot.setDriveMotors(DriveLeftPower, DriveRightPower, DriveLeftPower, DriveRightPower);
        }
    }
*/
    }}
