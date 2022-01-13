package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.*;

@Autonomous(name="Bernardo_2_RedAutonomous", group="ChampBot")

public class Bernardo_2_RedAutonomous extends LinearOpMode {
    ChampBot_v2 robot = new ChampBot_v2();
    private ElapsedTime runtime = new ElapsedTime();
    static final double tickCount = 537.7;
    static final double wheelDiameter = 3.78; //in inches
    static final double countsPerInch = tickCount/(wheelDiameter*3.1415);
    static final double driveSpeed = 1.0;
    static final double strafeSpeed = 0.5;
    public DcMotor DriveFrontLeft; //:D
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackLeft;
    public DcMotor DriveBackRight;
    public enum Direction {
        left, right;
    }

    @Override
    public void runOpMode() {
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

        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d");
        robot.DriveFrontLeft.getCurrentPosition();
        robot.DriveFrontRight.getCurrentPosition();
        robot.DriveBackLeft.getCurrentPosition();
        robot.DriveBackRight.getCurrentPosition();
        telemetry.update();

        waitForStart();

        encoderDrive(.5,-14,-14,3.0);
        robot.ArmServo.setPosition(0);
        sleep(750);
        encoderStrafe(.5,20,Direction.left,3.0); //straft left
        sleep(750);
        robot.IntakeWheel.setPower(.4);
        sleep(2000);//drop block
        robot.IntakeWheel.setPower(0);
        encoderStrafe(.5,15,Direction.right,3.0); //strafe right
        encoderStrafe(.2,10,Direction.right,3.0); //align against wall
        robot.ArmServo.setPosition(0.34); //position arm to straight
        sleep(750);
        encoderDrive(.5,23,23,3.0); //dirve forward a bit
        //need to set arm to correct hight to intake block
        robot.IntakeWheel.setPower(-.4);
        encoderDriveAndIntake(.2,15,15,3.0); //drive forward slowly while spinning intake wheel
        robot.IntakeWheel.setPower(0);
        //need to set arm back to original height
        encoderDrive(-.5,-38,-38,4.0); //dirve backwards to original position

        //repeat

        /* old code without cycle:
        robot.ArmServo.setPosition(0.34);
        sleep(250);
        encoderDrive(.5, 49, 49, 3.0);
        encoderTurn(.5,1,Direction.right,1.0);
        encoderDrive(.5, -13.5, -13.5, 2.0);
        robot.ArmServo.setPosition(1);
        sleep(500);
        robot.IntakeWheel.setPower(.4);
        sleep(750);
        robot.IntakeWheel.setPower(0);
        robot.ArmServo.setPosition(.34);
        encoderDrive(.5,-36.5,-36.5 ,3.0);
        encoderTurn(.5,1.3,Direction.left, 1.0);
        encoderDrive(.3,-41,-41, 3.0);
        //spin carousel wheel
        encoderTurn(.5,1.1,Direction.right,1.0);
        encoderDrive(.5, 55,55,4.0);
        encoderStrafe(.5,25,Direction.right,3.0);
        encoderDrive(.5, 22, 22, 3.0);
        encoderStrafe(.5,17, Direction.left, 2.0);
*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderDrive (double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = robot.DriveFrontLeft.getCurrentPosition() + (int) (1.25 * leftInches * countsPerInch);
            newFrontRightTarget = robot.DriveFrontRight.getCurrentPosition() + (int) (1.25 * rightInches * countsPerInch);
            newBackLeftTarget = robot.DriveBackLeft.getCurrentPosition() + (int) (1.25 * leftInches * countsPerInch);
            newBackRightTarget = robot.DriveBackRight.getCurrentPosition() + (int) (1.25 * rightInches * countsPerInch);

            robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
            robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
            robot.DriveBackRight.setTargetPosition(newBackRightTarget);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.DriveFrontLeft.setPower(Math.abs(speed));
            robot.DriveFrontRight.setPower(Math.abs(speed));
            robot.DriveBackLeft.setPower(Math.abs(speed));
            robot.DriveBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.DriveFrontLeft.isBusy() && robot.DriveFrontRight.isBusy() && robot.DriveBackLeft.isBusy() && robot.DriveBackRight.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.DriveFrontLeft.setPower(0);
            robot.DriveFrontRight.setPower(0);
            robot.DriveBackLeft.setPower(0);
            robot.DriveBackRight.setPower(0);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void encoderDriveAndIntake (double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = robot.DriveFrontLeft.getCurrentPosition() + (int) (1.25 * leftInches * countsPerInch);
            newFrontRightTarget = robot.DriveFrontRight.getCurrentPosition() + (int) (1.25 * rightInches * countsPerInch);
            newBackLeftTarget = robot.DriveBackLeft.getCurrentPosition() + (int) (1.25 * leftInches * countsPerInch);
            newBackRightTarget = robot.DriveBackRight.getCurrentPosition() + (int) (1.25 * rightInches * countsPerInch);

            robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
            robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
            robot.DriveBackRight.setTargetPosition(newBackRightTarget);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.DriveFrontLeft.setPower(Math.abs(speed));
            robot.DriveFrontRight.setPower(Math.abs(speed));
            robot.DriveBackLeft.setPower(Math.abs(speed));
            robot.DriveBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.DriveFrontLeft.isBusy() && robot.DriveFrontRight.isBusy() && robot.DriveBackLeft.isBusy() && robot.DriveBackRight.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }

            robot.DriveFrontLeft.setPower(0);
            robot.DriveFrontRight.setPower(0);
            robot.DriveBackLeft.setPower(0);
            robot.DriveBackRight.setPower(0);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);
        }
    }

    public void encoderStrafe (double strafeSpeed, double inches, Direction direction, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = robot.DriveFrontLeft.getCurrentPosition() + (int) (inches * countsPerInch);
            newFrontRightTarget = robot.DriveFrontRight.getCurrentPosition() + (int) (inches * countsPerInch);
            newBackLeftTarget = robot.DriveBackLeft.getCurrentPosition() + (int) (inches * countsPerInch);
            newBackRightTarget = robot.DriveBackRight.getCurrentPosition() + (int) (inches * countsPerInch);

            robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
            robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
            robot.DriveBackRight.setTargetPosition(newBackRightTarget);

            if (direction == Direction.right) {
                robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
                robot.DriveFrontRight.setTargetPosition(-newFrontRightTarget);
                robot.DriveBackLeft.setTargetPosition(-newBackLeftTarget);
                robot.DriveBackRight.setTargetPosition(newBackRightTarget);
            }else {
                robot.DriveFrontLeft.setTargetPosition(-newFrontLeftTarget);
                robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
                robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
                robot.DriveBackRight.setTargetPosition(-newBackRightTarget);
            }

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.DriveFrontLeft.setPower(strafeSpeed);
            robot.DriveFrontRight.setPower(strafeSpeed);
            robot.DriveBackLeft.setPower(strafeSpeed);
            robot.DriveBackRight.setPower(strafeSpeed);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.DriveFrontLeft.isBusy() && robot.DriveFrontRight.isBusy() && robot.DriveBackLeft.isBusy() && robot.DriveBackRight.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }

            robot.DriveFrontLeft.setPower(0);
            robot.DriveFrontRight.setPower(0);
            robot.DriveBackLeft.setPower(0);
            robot.DriveBackRight.setPower(0);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void encoderTurn (double speed, double numberOfTurns,   Direction direction, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        double inchCount = 4.22 * Math.PI;

        if (opModeIsActive()) {
            newFrontLeftTarget = robot.DriveFrontLeft.getCurrentPosition() + (int) (inchCount * countsPerInch * numberOfTurns);
            newFrontRightTarget = robot.DriveFrontRight.getCurrentPosition() + (int) (inchCount * countsPerInch * numberOfTurns);
            newBackLeftTarget = robot.DriveBackLeft.getCurrentPosition() + (int) (inchCount * countsPerInch * numberOfTurns);
            newBackRightTarget = robot.DriveBackRight.getCurrentPosition() + (int) (inchCount * countsPerInch * numberOfTurns);

            if(direction == Direction.right) {
                robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
                robot.DriveFrontRight.setTargetPosition(-newFrontRightTarget);
                robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
                robot.DriveBackRight.setTargetPosition(-newBackRightTarget);
            } else {
                robot.DriveFrontLeft.setTargetPosition(-newFrontLeftTarget);
                robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
                robot.DriveBackLeft.setTargetPosition(-newBackLeftTarget);
                robot.DriveBackRight.setTargetPosition(newBackRightTarget);
            }


            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.DriveFrontLeft.setPower(Math.abs(speed));
            robot.DriveFrontRight.setPower(Math.abs(speed));
            robot.DriveBackLeft.setPower(Math.abs(speed));
            robot.DriveBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.DriveFrontLeft.isBusy() && robot.DriveFrontRight.isBusy() && robot.DriveBackLeft.isBusy() && robot.DriveBackRight.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.DriveFrontLeft.setPower(0);
            robot.DriveFrontRight.setPower(0);
            robot.DriveBackLeft.setPower(0);
            robot.DriveBackRight.setPower(0);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void setArm (double position) {
        
    }
    /*public void encoderArm (double speed, double armTickCount, double timeoutS) {
        int newArmTarget;

        if (opModeIsActive()) {
            newArmTarget = robot.ArmMotor.getCurrentPosition() + (int) (armTickCount);

            robot.ArmMotor.setTargetPosition(newArmTarget);

            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.ArmMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.ArmMotor.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d", newArmTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.ArmMotor.setPower(0);

            robot.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    } */
}
