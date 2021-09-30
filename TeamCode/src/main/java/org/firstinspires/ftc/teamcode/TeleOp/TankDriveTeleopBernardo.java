package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.ChampBot;


@TeleOp(name="TankDriveBernardo", group="ChampBot")
public class TankDriveTeleopBernardo extends OpMode {
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
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
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

        //Strafe Control
        if (RightStrafe) {
            // to right strafe, right motors towards each other, left motors away from each other
            robot.frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(1);
            robot.rearLeftDrive.setPower(1);
            robot.rearRightDrive.setPower(1);
        } else if (LeftStrafe) {
            // opposite of right strafe
            robot.frontLeftDrive.setPower(-1);
            robot.frontRightDrive.setPower(-1);
            robot.rearLeftDrive.setPower(-1);
            robot.rearRightDrive.setPower(-1);
        }
        robot.setDriveMotors(-DriveLeftPower, -DriveRightPower, DriveLeftPower, -DriveRightPower);

        //import odometry into the loop
        robot.odometry();
        telemetry.addData("LRA","%6d %6d %6d", robot.currentLeftPosition, robot.currentRightPosition, robot.currentAuxPosition);
        telemetry.addData("xyh", "%6.1f cm 6.1f cm %6.1f deg", robot.pos.x, robot.pos.y, Math.toDegrees(robot.pos.h));
        //telemetry.addData("loop", ".1f ms", timer.milliseconds);
        //timer.reset();

    }
}