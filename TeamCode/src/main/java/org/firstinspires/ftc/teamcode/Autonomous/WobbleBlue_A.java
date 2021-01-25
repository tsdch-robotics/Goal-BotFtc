
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
    static DcMotor FrontLeftMotor;
    static DcMotor FrontRightMotor;
    static DcMotor BackLeftMotor;
    static DcMotor BackRightMotor;
    static DcMotor ArmMotor;
    static DcMotor WheelMotor;
    static DcMotor LauncherMotor;
    private static HardwareMap hardwareMap;
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
        //sleep(250);//wait for 1 sec
        DriveRobot(650, 0,0,0, 0, -1,0,0);//move the arm down to horizontal
        //sleep(250);//wait for 1 sec
        DriveRobot(260, .5,.5,.5, .5, 0,0,0);//move forward 5 in
        //sleep(250);//wait for 1 sec
        DriveRobot(650, 0, 0,0,0, 1,0,0);//move the arm up to vertical
       // sleep(250);//wait for 1 sec
        DriveRobot(2900, 0.5, .5,-.5,-0.5, 0,0,0);//turn back
        //sleep(250);//wait for 1 sec
        DriveRobot(4195, .5, .5,.5,.5, 0,0,0);//move forward 80.75 in
       // sleep(250);//wait for 1 sec
        DriveRobot(650, 0,0,0, 0, -1,0,0);//move the arm down to horizontal
       // sleep(250);//wait for 1 sec
        DriveRobot(520, -.5, -.5,-.5,-.5, 0,0,0);//move backwards 10 in
       // sleep(250);//wait for 1 sec
        DriveRobot(650, 0, 0,0,0, 1,0,0);//move the arm up to vertical
       // sleep(250);//wait for 1 esc
        DriveRobot(3935, -.5,-.5,-.5, -.5, 0,0,0);//move backward 80.75 in
       // sleep(250);//wait for 1 sec
        DriveRobot(2900, -.5,-.5,.5, .5, 0,0,0);//turn back
       // sleep(250);//wait for 1 sec
        DriveRobot(650, 0, 0,0,0, -1,0,0);//move the arm down to horizontal
      //  sleep(250);//wait for 1 sec
        DriveRobot(260, .5, .5,.5,.5, 0,0,0);//move forward 5 in
        DriveRobot(650, 0,0,0, 0, 1,0,0);//move the arm up to vertical
    }


    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException {
        WobbleBlue_A();
    }


    private static void DriveRobot(int milliseconds, double LeftFrontPower, double LeftBackPower, double RightFrontPower, double RightBackPower, double ArmPower, double WheelPower, double LauncherPower) {

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