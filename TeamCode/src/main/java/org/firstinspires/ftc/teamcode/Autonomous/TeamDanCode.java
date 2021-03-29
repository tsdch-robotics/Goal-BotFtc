package org.firstinspires.ftc.teamcode.Autonomous;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class TeamDanCode extends LinearOpMode {
    DcMotor FrontLeftMotor;
    DcMotor FrontRightMotor;
    Servo servo;
    Servo servo2;

    public void runOpMode() {
        FrontLeftMotor = hardwareMap.dcMotor.get("DriveFrontLeft");
        FrontRightMotor = hardwareMap.dcMotor.get("DriveFrontRight");
        servo = hardwareMap.get(Servo.class, "Claw");
        servo2 = hardwareMap.get(Servo.class, "Claw2");
        DriveRobot(500, 0.75, 0.75, 0, 0);//move forward
        sleep(250);//wait for 0.25 sec
        DriveRobot(250, 0,0, .5, -.5);//grab
    }


    private void DriveRobot(int milliseconds, double LeftFrontPower, double RightFrontPower, double position1, double position2) {
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        //Wait for start button


        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();


        // set both motors to x power.

        FrontLeftMotor.setPower(LeftFrontPower);
        FrontRightMotor.setPower(RightFrontPower);
        servo.setPosition(position1);
        servo2.setPosition(position2);


        sleep(milliseconds);        // wait for x seconds.

        // set motor power to zero to stop motors.

        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
    }
}