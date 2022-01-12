package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.ChampBot_v2;

public class PIDController extends LinearOpMode {

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ChampBot_v2 robot = new ChampBot_v2();

        //waitforStart();

        while (opModeIsActive()) {

        }
    }

    public double PIDControl(double reference, double state) {
        double error =reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative *Kd) + (integralSum * Ki);
        return output;
    }
}
