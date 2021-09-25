package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Jeff_Autonomous extends LinearOpMode {
    DcMotor FrontLeftMotor;
    DcMotor FrontRightMotor;
    DcMotor BackRightMotor;

    public void runOpMode() {
        FrontLeftMotor = hardwareMap.dcMotor.get("LeftMotor");
        FrontRightMotor = hardwareMap.dcMotor.get("RightMotor");
        BackRightMotor = hardwareMap.dcMotor.get("BackRightMotor");
        DriveRobot(500, 0.75, 0.75);//move forward
        sleep(250);//wait for 0.25 sec
        DriveRobot(400, 1, 0);
        sleep(250);
        DriveRobot(500,-1,-1);
    }


    public void DriveRobot(int milliseconds, double LeftPower, double RightPower) {
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        //Wait for start button


        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();


        // set both motors to x power.

        FrontLeftMotor.setPower(LeftPower);
        FrontRightMotor.setPower(RightPower);

        sleep(milliseconds);        // wait for x seconds.

        // set motor power to zero to stop motors.

        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackRightMotor.setPower(0);
    }
}

