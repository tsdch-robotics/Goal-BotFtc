
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.

@Autonomous(name="Wobble_Blue_A", group="ChampBot")
//@Disabled
public class WobbleBlue_A extends LinearOpMode {
    public static DcMotor FrontLeftMotor;
    public static DcMotor FrontRightMotor;
    public static DcMotor BackLeftMotor;
    public static DcMotor BackRightMotor;
    public static DcMotor ArmMotor;
    public static DcMotor WheelMotor;
    public static DcMotor LauncherMotor;
    public static HardwareMap hardwareMap;
    int milliseconds = 0;
    double LeftPower = 0;
    double RightPower = 0;

    public static void WobbleBlue_A() {
        FrontLeftMotor = hardwareMap.dcMotor.get("DriveFrontLeft");
        FrontRightMotor = hardwareMap.dcMotor.get("DriveFrontRight");
        BackLeftMotor = hardwareMap.dcMotor.get("DriveBackLeft");
        BackRightMotor = hardwareMap.dcMotor.get("DriveBackRight");
        ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        WheelMotor = hardwareMap.dcMotor.get("WheelMotor");
        LauncherMotor = hardwareMap.dcMotor.get("LauncherMotor");
        BackRightMotor.setDirection(DcMotor.Direction.REVERSE);
        DriveRobot(3935, .5,.5,.5, .5, 0,-1,-1);//move forward 80.75 in
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(650, 0,0,0, 0, -1,0,0);//move the arm down to horizontal
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(260, .5,.5,.5, .5, 0,0,0);//move forward 5 in
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(650, 0, 0,0,0, 1,0,0);//move the arm up to vertical
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(2900, 0.5, .5,-.5,-0.5, 0,0,0);//turn back
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(4195, .5, .5,.5,.5, 0,0,0);//move forward 80.75 in
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(650, 0,0,0, 0, -1,0,0);//move the arm down to horizontal
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(520, -.5, -.5,-.5,-.5, 0,0,0);//move backwards 10 in
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(650, 0, 0,0,0, 1,0,0);//move the arm up to vertical
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(3935, -.5,-.5,-.5, -.5, 0,0,0);//move backward 80.75 in
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(2900, -.5,-.5,.5, .5, 0,0,0);//turn back
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(650, 0, 0,0,0, -1,0,0);//move the arm down to horizontal
        DriveRobot(250,0,0,0,0,0,0,0);//wait for 0.25 sec
        DriveRobot(260, .5, .5,.5,.5, 0,0,0);//move forward 5 in
        DriveRobot(650, 0,0,0, 0, 1,0,0);//move the arm up to vertical
    }


    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException {
        WobbleBlue_A();
    }


    public static void DriveRobot(int milliseconds, double LeftFrontPower, double LeftBackPower, double RightFrontPower, double RightBackPower, double ArmPower, double WheelPower, double LauncherPower) {

        // wait for start button.



        // set both motors to x power.

        FrontLeftMotor.setPower(LeftFrontPower);
        FrontRightMotor.setPower(RightFrontPower);
        BackLeftMotor.setPower(LeftBackPower);
        BackRightMotor.setPower(RightBackPower);
        ArmMotor.setPower(ArmPower);
        WheelMotor.setPower(WheelPower);
        LauncherMotor.setPower(LauncherPower);


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