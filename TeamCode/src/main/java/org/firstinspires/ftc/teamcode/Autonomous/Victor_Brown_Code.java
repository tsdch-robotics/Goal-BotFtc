package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class Victor_Brown_Code extends LinearOpMode {
    DcMotor left_motor;
    DcMotor right_motor;

    public void runOpMode() {
        left_motor = hardwareMap.dcMotor.get("LeftMotor");
        left_motor = hardwareMap.dcMotor.get("RightMotor");
        DriveRobot(500, 0.75, 0.75);//move forward
        sleep(250);//wait for 0.25 sec
        DriveRobot(400, 1, 0); //turn right
        sleep(250);
        DriveRobot(500,-1,-1); //back up to the left

        DriveRobot(400, 1, 0); //turn right
        sleep (250);
        DriveRobot(400, 0, 1); //turn right
        sleep (250);
        DriveRobot(400, 1, 0); //turn right
        sleep (250);
        DriveRobot(400, 0, 1); //turn right
        sleep (250);
        DriveRobot(700, -1, 1);
    }
    public abstract void DriveRobot(int milliseconds, double LeftPower, double RightPower);
}
