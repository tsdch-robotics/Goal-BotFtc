
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.

@Autonomous(name="Wobble_Blue_C", group="ChampBot")
//@Disabled
public class  WobbleBlue_C extends LinearOpMode {
    DcMotor FrontLeftMotor;
    DcMotor FrontRightMotor;
    DcMotor BackLeftMotor;
    DcMotor BackRightMotor;
    DcMotor ArmMotor;
    DcMotor WheelMotor;
    DcMotor LauncherMotor;
    int milliseconds = 0;
    double LeftPower = 0;
    double RightPower = 0;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeftMotor = hardwareMap.dcMotor.get("DriveFrontLeft");
        FrontRightMotor = hardwareMap.dcMotor.get("DriveFrontRight");
        BackLeftMotor = hardwareMap.dcMotor.get("DriveBackLeft");
        BackRightMotor = hardwareMap.dcMotor.get("DriveBackRight");
        ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        WheelMotor = hardwareMap.dcMotor.get("WheelMotor");
        LauncherMotor = hardwareMap.dcMotor.get("LauncherMotor");
        BackRightMotor.setDirection(DcMotor.Direction.REVERSE);
        DriveRobot(571, -.5,.5,.5,-.5,0,0,0);//strafe left 11 in
        sleep(250);
        DriveRobot(6117, .5,.5,.5,.5,0,0,0);//move forward 117.75 in
        sleep(250);
        DriveRobot(650,0,0,0,0,-1,0,0);//move the arm down to horizontal
        sleep(250);
        DriveRobot(260, .5,.5,.5,.5,0,0,0);//move forward 5 in
        sleep(250);
        DriveRobot(1753, .5,-.5,-.5,.5,0,0,0);//strafe right 33.75 in
        sleep(250);
        DriveRobot(650,0,0,0,0,1,0,0);//move the arm up to vertical
        sleep(250);
        DriveRobot(2338, -0.5, -.5,-.5,-.5,0,0,.95);//movebackward for 45 in
        sleep(250);//wait for 0.25 sec
        DriveRobot(1000, 0,0,0, 0, 0,-0.5,.95);//launch ring
        DriveRobot(250, 0,0,0, 0, 0,0,.95);//sleep while launcher move
        DriveRobot(415, .5,-.5,-.5,.5,0,0,.95);//strafe right 8 in
        DriveRobot(250, 0,0,0, 0, 0,0,.95);//sleep while launcher move
        DriveRobot(1000, 0,0,0, 0, 0,-0.5,.95);//launch ring
        DriveRobot(250, 0,0,0, 0, 0,0,.95);//sleep while launcher move
        DriveRobot(415, .5,-.5,-.5,.5,0,0,.95);//strafe right 8 in
        DriveRobot(250, 0,0,0, 0, 0,0,.95);//sleep while launcher move
        DriveRobot(1000, 0,0,0, 0, 0,-0.5,.95);//launch ring
        DriveRobot(300, .5,.5,.5, .5, 0,0,0);//move forward a bit to park
    }


    private void DriveRobot(int milliseconds, double LeftFrontPower, double LeftBackPower, double RightFrontPower, double RightBackPower, double ArmPower, double WheelPower, double LauncherPower) {
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set motors to x power.

        FrontLeftMotor.setPower(LeftFrontPower);
        FrontRightMotor.setPower(RightFrontPower);
        BackLeftMotor.setPower(LeftBackPower);
        BackRightMotor.setPower(RightBackPower);
        ArmMotor.setPower(ArmPower);
        WheelMotor.setPower(WheelPower);
        LauncherMotor.setPower(LauncherPower);


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