package org.firstinspires.ftc.teamcode.TeleOp;

import android.print.PrinterId;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Autonomous.ChampBot_v2;

public class PIDController extends LinearOpMode {

    public DcMotor DriveFrontLeft;
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackLeft;
    public DcMotor DriveBackRight;

    private BNO055IMU imu; //model number of inertial measurment unit in REV Expansion hub

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override

    public void runOpMode() throws InterruptedException {
        ChampBot_v2 robot = new ChampBot_v2();
        robot.init(hardwareMap);

        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //imu = hardwareMap.get(BNO055IMU.class, imu);

        //waitforStart();

        while (opModeIsActive()) {
            double power = PIDControl(100, DriveFrontLeft.getCurrentPosition());
            DriveFrontLeft.setPower(power);
        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}
