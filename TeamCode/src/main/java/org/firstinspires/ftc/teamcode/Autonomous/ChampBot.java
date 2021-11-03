package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Vision imports
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/*
 * This is NOT an opmode. This file defines all the hardware on the robot
 * and some common helper functions (stop motors, reset encoders, etc.)
 */
public class ChampBot {
    public static final Double TRIGGER_THRESHOLD = 0.5;//gamepad trigger
    //Drive Motors
    public DcMotor DriveFrontLeft;
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackLeft;
    public DcMotor DriveBackRight;
    //Odometry Encoders
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;
//code time! :)
    private HardwareMap hardwareMap;
    public void init(HardwareMap ahwMap){
        hardwareMap=ahwMap;
        //configure the drive motors
        DriveFrontLeft=hardwareMap.dcMotor.get("DriveFrontLeft");
        DriveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveFrontRight=hardwareMap.dcMotor.get("DriveFrontRight");
        DriveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveBackLeft=hardwareMap.dcMotor.get("DriveBackLeft");
        DriveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveBackRight=hardwareMap.dcMotor.get("DriveBackRight");
        DriveBackRight.setDirection(DcMotor.Direction.REVERSE);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //shadow the motors with the odo encoders
        encoderLeft=DriveFrontLeft;
        encoderRight=DriveBackRight;
        encoderAux=DriveFrontRight;

        stop();
        resetDriveEncoders();
    }
    //a function to reset encoder
    public void resetDriveEncoders(){
        DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //the stop function

    public void stop() {
        DriveFrontLeft.setPower(0);
        DriveFrontRight.setPower(0);
        DriveBackRight.setPower(0);
        DriveBackLeft.setPower(0);
    }

    public void setDriveMotors(double FrontL, double FrontR, double BackL, double BackR) {
        DriveFrontLeft.setPower(FrontL);
        DriveFrontRight.setPower(FrontR);
        DriveBackLeft.setPower(-BackL);
        DriveBackRight.setPower(BackR);
    }
    //constants that define the geometry of the robot:
    final static double L = 13.5; //distance between encoder 1 and 2 in cm
    final static double B = 5.31; //Distance between the mid-point of the encoder 1, encoder 2 and encoder 3 in cm
    final static double R = 7.72; //Wheel Radius in cm
    final static double N = 8192; //encoder ticks per revolution, REV encoder
    final static double cm_per_tick = 2.0 * Math.PI*R/N;

    //keep track of the odometry encoders between updates;
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;
    /*************************************************************************************
     * Odometry
     * Notes:
     * n1, n2, n3 are encode values for the left, right, and back (aux) omniwheels
     * dn1, dn2, dn3 are the differences of encoder values between two reads
     * dx, dy, dtheta describe the robot movement between two reads (in robot coordinates)
     * X, Y, Theta are the coordinates on the field and the heading of the robot.
     *************************************************************************************/
    //XyhVector is a tuple (x,y,h) where h is the angle, the heading of the robot.
    public XyhVector START_POS = new XyhVector(213,102, Math.toRadians(-174));
    public XyhVector pos = new XyhVector(START_POS);

    public void odometry(){

        //getting the position information from the encoders
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = -encoderLeft.getCurrentPosition();
        currentAuxPosition = encoderAux.getCurrentPosition();

        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        //record the change in position of the robot between two measurements over a very small time interval
        double dtheta = cm_per_tick * (dn2-dn1)/L;
        double dx = cm_per_tick * (dn1+dn2)/2.0;
        double dy = cm_per_tick * (dn3-(dn2-dn1) * B / L);

        //trnasfer the robot coordinate into the field coordinate
        double theta = pos.h +(dtheta / 2.0);
        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;
    }
}