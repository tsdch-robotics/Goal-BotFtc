
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.

@Autonomous(name="Wobble_Blue_A", group="ChampBot")
//@Disabled
public class WobbleBlue_A extends LinearOpMode {
    DcMotor FrontLeftMotor;
    DcMotor FrontRightMotor;
    DcMotor BackLeftMotor;
    DcMotor BackRightMotor;
    DcMotor ArmMotor;
    DcMotor WheelMotor;
    DcMotor LauncherMotor;
    Servo servo;

    public void runOpMode() throws InterruptedException {
        FrontLeftMotor = hardwareMap.dcMotor.get("DriveFrontLeft");
        FrontRightMotor = hardwareMap.dcMotor.get("DriveFrontRight");
        BackLeftMotor = hardwareMap.dcMotor.get("DriveBackLeft");
        BackRightMotor = hardwareMap.dcMotor.get("DriveBackRight");
        ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        WheelMotor = hardwareMap.dcMotor.get("WheelMotor");
        LauncherMotor = hardwareMap.dcMotor.get("LauncherMotor");
        BackRightMotor.setDirection(DcMotor.Direction.REVERSE);
        servo = hardwareMap.get(Servo.class, "Claw");
        DriveRobot(500, 0, 0, 0, 0, 0, 0, 0, 1);//grab
        sleep(250);//wait for 0.25 sec
        DriveRobot(400, 0, 0, 0, 0, -1, 0, 0, 1);//move the arm down half way
        sleep(250);//wait for 0.25 sec
        DriveRobot(2000, 1, 1, 1, 1, 0, 0, 0, 1);//move forward 90.75 in
        sleep(250);//wait for 0.25 sec
        DriveRobot(600, 0, 0, 0, 0, -1, 0, 0, 1);//move the arm down to horizontal
        sleep(250);//wait for 0.25 sec
        DriveRobot(600, 0, 0, 0, 0, 0, 0, 0, 0);//open claw
        DriveRobot(600, 0, 0, 0, 0, 1, 0, 0, 0);//move the arm up to vertical
        DriveRobot(800, -0.5, -.5, 0, 0, 0, 0, 0, 0);//tilt
        sleep(250);//wait for 0.25 sec
        DriveRobot(1850, -1, -1, -1, -1, 0, 0, 0, 0);//move backwards to wobble
        sleep(250);//wait for 0.25 sec
        DriveRobot(350, 0,0,0, 0, -1,0,0,0);//move the arm down all the way
        sleep(500);
        DriveRobot(300,0.3,0.3,0.3,0.3,0,0,0,0);//drive forward
        DriveRobot(260, 0,0,0,0,0,0,0,1);//close claw
        sleep(500);
        DriveRobot(400,0,0,0,0,1,0,0,1);//move the arm up to vertical
        DriveRobot(1700, 1, 1, 1, 1, 0, 0, 0, 1);//swerve forward
        DriveRobot(400, 0, 0, 0, 0, -1, 0, 0, 1);//move the arm down to horizontal
        sleep(250);//wait for 0.25 sec
        DriveRobot(600, 0, 0, 0, 0, 0, 0, 0, 0);//open claw
        DriveRobot(600, 0, 0, 0, 0, 1, 0, 0, 0);//move the arm up to vertical
        DriveRobot(1000, -0.1, -0.1, -0.6, -0.6, 0, 0, 0, 0);//tilt
        DriveRobot(1000, 0,0,0, 0, 0,0,0.83,1);//spin launcher
        DriveRobot(4000, 0,0,0, 0, 0,0.25,0.83,1);//launch ring
        DriveRobot(500, 1,1,1,1,1,0,0,1);//drive forward
    }


    // called when init button is  pressed.


    public void DriveRobot(int milliseconds, double LeftFrontPower, double LeftBackPower, double RightFrontPower, double RightBackPower, double ArmPower, double WheelPower, double LauncherPower, double position) {
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();


        // set both motors to x power.

        FrontLeftMotor.setPower(LeftFrontPower);
        FrontRightMotor.setPower(RightFrontPower);
        BackLeftMotor.setPower(LeftBackPower);
        BackRightMotor.setPower(RightBackPower);
        ArmMotor.setPower(ArmPower);
        WheelMotor.setPower(WheelPower);
        LauncherMotor.setPower(LauncherPower);
        servo.setPosition(position);

        sleep(milliseconds);        // wait for x seconds.

        // set motor power to zero to stop motors.

        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);
        ArmMotor.setPower(0);
        LauncherMotor.setPower(0);
        WheelMotor.setPower(0);
    }
}