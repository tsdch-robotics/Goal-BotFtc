package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.*;

@Autonomous(name="TestAuto2022", class="ChampBot")

public class TestAuto2022 extends LinearOpMode {
    ChampBot robot = new ChampBot();

    public DcMotor DriveFrontLeft;
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackRight;
    public DcMotor DriveBackLeft;
    public DcMotor duck_wheel;

    public void runOpMode {
        robot.init(hardwareMap);

        telemetry.addData("Status: ", "Resetting Encoders");
        telemetry.update();

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d");
        robot.DriveFrontLeft.getCurrentPosition();
        robot.DriveFrontRight.getCurrentPosition();
        robot.DriveBackLeft.getCurrentPosition();
        robot.DriveBackRight.getCurrentPosition();
        telemetry.update();

        waitForStart();

        standardDrive(0.5);
    }

    public void standardDrive(double speed) {


        if (opModeIsActive()) {
            robot.DriveFrontLeft.setPower(speed);
            robot.DriveFrontRight.setPower(speed);
            robot.DriveBackLeft.setPower(speed);
            robot.DriveBackRight.setPower(speed);
        }


    }

}