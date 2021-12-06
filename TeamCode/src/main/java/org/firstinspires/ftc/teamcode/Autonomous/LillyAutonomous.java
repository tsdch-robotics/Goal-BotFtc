package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.*;

@Autonomous(name="LillyAutonomous", group="ChampBot")

public class LillyAutonomous extends LinearOpMode {
    ChampBot robot = new ChampBot();
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
        left,right;
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

        //encoderStrafe(strafeSpeed, 21,Direction.left, 2.0);
        //encoderDrive(driveSpeed, 19, 19, 3.0);
        //Arm Placement
        //encoderDrive(driveSpeed, -18, -18, 3.0);
        //encoderTurn(driveSpeed, 1, Direction.right, 1.0);
        //encoderDrive(driveSpeed, 58, 58, 5.0);

        encoderDrive(driveSpeed, 15, 15, 3.0);
        encoderTurn(driveSpeed, 1, Direction.left, 1.0);
        /*encoderDrive(driveSpeed, 16.5, 16.5, 3.0);
        encoderTurn(driveSpeed, 1, Direction.right, 1.0);
        encoderDrive(driveSpeed, 4, 4, 1.0);
        // place block
        encoderDrive(driveSpeed, -4,-4, 1.0);
        encoderTurn(driveSpeed, 1, Direction. right, 1.0);
        encoderDrive(driveSpeed, 16.5, 16.5, 3.0);
        encoderStrafe(strafeSpeed, 19, Direction.right, 3.0);
        encoderDrive(driveSpeed, 36,36,4.0); */



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void encoderDrive (double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = robot.DriveFrontLeft.getCurrentPosition() + (int) (leftInches * countsPerInch);
            newFrontRightTarget = robot.DriveFrontRight.getCurrentPosition() + (int) (rightInches * countsPerInch);
            newBackLeftTarget = robot.DriveBackLeft.getCurrentPosition() + (int) (leftInches * countsPerInch);
            newBackRightTarget = robot.DriveBackRight.getCurrentPosition() + (int) (rightInches * countsPerInch);

            robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
            robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
            robot.DriveBackRight.setTargetPosition(newBackRightTarget);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.DriveFrontLeft.setPower(Math.abs(driveSpeed));
            robot.DriveFrontRight.setPower(Math.abs(driveSpeed));
            robot.DriveBackLeft.setPower(Math.abs(driveSpeed));
            robot.DriveBackRight.setPower(Math.abs(driveSpeed));

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
                robot.DriveFrontLeft.setPower(Math.abs(strafeSpeed));
                robot.DriveFrontRight.setPower(Math.abs(strafeSpeed));
                robot.DriveBackLeft.setPower(Math.abs(strafeSpeed));
                robot.DriveBackRight.setPower(Math.abs(strafeSpeed));

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
        double inchCount = 6.55 * Math.PI;

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

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    }


