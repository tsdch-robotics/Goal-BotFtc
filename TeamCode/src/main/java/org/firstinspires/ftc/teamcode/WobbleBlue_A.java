
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        //FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //FrontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotor.Direction.REVERSE);
        DriveRobot(3935, .5,.5,0);//move forward 80.75 in
        sleep(250);//wait for 1 sec
        DriveRobot(650,0,0,-1);//move the arm down to horizontal
        sleep(250);//wait for 1 sec
        DriveRobot(260, .5,.5,0);//move forward 5 in
        sleep(250);//wait for 1 sec
        DriveRobot(650,0,0,1);//move the arm up to vertical
        sleep(250);//wait for 1 sec
        DriveRobot(2900, 0.5, -0.5, 0);//turn back
        sleep(250);//wait for 1 sec
        DriveRobot(4195, .5,.5,0);//move forward 80.75 in
        sleep(250);//wait for 1 sec
        DriveRobot(650,0,0,-1);//move the arm down to horizontal
        sleep(250);//wait for 1 sec
        DriveRobot(520, -.5,-.5,0);//move backwards 10 in
        sleep(250);//wait for 1 sec
        DriveRobot(650,0,0,1);//move the arm up to vertical
        sleep(250);//wait for 1 esc
        DriveRobot(3935, -.5,-.5,0);//move backward 80.75 in
        sleep(250);//wait for 1 sec
        DriveRobot(2900, -0.5, 0.5, 0);//turn back
        sleep(250);//wait for 1 sec
        DriveRobot(650,0,0,-1);//move the arm down to horizontal
        sleep(250);//wait for 1 sec
        DriveRobot(260, .5,.5,0);//move forward 5 in
        DriveRobot(650,0,0,1);//move the arm up to vertical
    }


    private void DriveRobot(int milliseconds, double LeftPower, double RightPower, double ArmPower) {
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to x power.

        FrontLeftMotor.setPower(LeftPower);
        FrontRightMotor.setPower(RightPower);
        BackLeftMotor.setPower(LeftPower);
        BackRightMotor.setPower(RightPower);
        ArmMotor.setPower(ArmPower);


        sleep(milliseconds);        // wait for x seconds.

        // set motor power to zero to stop motors.

        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);
        ArmMotor.setPower(0);
    }
}