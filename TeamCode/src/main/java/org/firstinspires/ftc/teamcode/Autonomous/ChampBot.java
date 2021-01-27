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
    // class variables for all hardware
    public DcMotor DriveFrontLeft;
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackLeft;
    public DcMotor DriveBackRight;
    //the intake motor
    public DcMotor WheelMotor;
    public DcMotor ArmMotor;
    public DcMotor IntakeMotor;
    //public DcMotor LauncherMotor;
    /*
    public DcMotor Arm;

    public Servo Wrist;
    public Servo Claw;

    public Servo PlatformServo;
    public Servo LockServo;
    */
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AYuwB/n/////AAABmc2iWLR8g0iipnUkJKVfgAYw+QI3BcT5KMR/SavKNiO/7h1HrtK20ekoQerKKc0YoamY11r9MOZzcgz6ku69rBwqrrl08VUqzKn+d49/pW3Gi6SseQMgb5piXwASgO9XHeqCFgmD+NkR52ta3MGEI8X6FGAt3uATqM20EPbIugPpnNjsdCgCav51jMCUI5kvgG4AjO4MIN/kPE4PlJ3ZUI7/lTSDZ8nImPoRuJQ9VWJrjOJzY6/ylE9V5j5r5nkixzVwLJ1GzA0vYsvFc+62J11ZuhiAoc1zxzpe8VK4ibSxwCP1lFRSg+6T8jiX4OXYnzovD4ghLc+0KXtF+hl9niNSkiBY7oaRYGwQW1MlgzJ9";
    //public VuforiaStuff vuforiaStuff;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public ChampBot() { }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap; // reference to hardware map

        // initialize drivetrain
        DriveFrontLeft = hwMap.dcMotor.get("DriveFrontLeft");
        DriveBackLeft = hwMap.dcMotor.get("DriveBackLeft");
        DriveFrontRight = hwMap.dcMotor.get("DriveFrontRight");
        DriveBackRight = hwMap.dcMotor.get("DriveBackRight");
        //initialize intake motor
        WheelMotor = hwMap.dcMotor.get("WheelMotor");
        ArmMotor = hwMap.dcMotor.get("ArmMotor");
        //IntakeMotor = hwMap.dcMotor.get("IntakeMotor");
        //LauncherMotor = hwMap.dcMotor.get("LauncherMotor");

        // reverse one side of the drivetrain so that directions are more natural
        DriveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        DriveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        DriveBackLeft.setDirection(DcMotor.Direction.REVERSE);

        // initialize arm
       /* Arm = hwMap.dcMotor.get("Arm");
        Arm.setDirection(DcMotor.Direction.REVERSE);

        // initialize servos
        Wrist = hwMap.servo.get("Wrist");
        Claw = hwMap.servo.get("Claw");
        PlatformServo = hwMap.servo.get("PlatformServo");
        LockServo = hwMap.servo.get("LockServo");
*/
        // move all motors/servos to their starting position
        //initAllServos();
       // stopAllMotors();

        // don't initialize gyro or vision unless an opmode specifically requests it!
    }

    public void visionInit() {
        WebcamName webcamName;
        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraName = webcamName;

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Initialize our block detection logic
        //vuforiaStuff = new VuforiaStuff(vuforia);
    }

    /**
     * Sets the drive motors to the specified power.
     * -1.0 moves the motor backwards; +1.0 moves the motor forwards
     * @param FrontL
     * @param FrontR
     * @param BackL
     * @param BackR
     */
    public void setDriveMotors(double FrontL, double FrontR, double BackL, double BackR) {
        DriveFrontLeft.setPower(FrontL);
        DriveFrontRight.setPower(FrontR);
        DriveBackLeft.setPower(BackL);
        DriveBackRight.setPower(BackR);
    }

    public void stopAllMotors() {
        DriveFrontLeft.setPower(0.0);
        DriveFrontRight.setPower(0.0);
        DriveBackLeft.setPower(0.0);
        DriveBackRight.setPower(0.0);
        WheelMotor.setPower(0.0);
        ArmMotor.setPower(0.0);

       // Arm.setPower(0.0);
    }

   /* public void initAllServos() {
        Wrist.setPosition(1.0); //in
        Claw.setPosition(0.0); // close
        PlatformServo.setPosition(0.0); //up
        LockServo.setPosition(0.0); //un-locked
    }
*/
    public void resetAllEncoders() {
        // reset drive encoders
        DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int inchesToEncoderCounts(double inches) {
        // CONSTANTS that only change when hardware changes are made to the robot
        final double countsPerShaftRotation = 1425.2; // only change this if you change what motor you're using
        final double shaftToWheelRatio = 24.0 / 16.0; // 1 turn of the motor shaft results in X turns of the wheel
        final double wheelDiameter = 4.0; // diameter of the wheel in inches

        // CALCULATIONS - don't change these!
        double wheelCircumference = Math.PI * wheelDiameter; // inches
        double countsPerWheelRotation = countsPerShaftRotation / shaftToWheelRatio;
        double countsPerInch = countsPerWheelRotation / wheelCircumference;

        return (int)(countsPerInch * inches);
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal) {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) index = -index;

        // index cannot exceed size of array minus 1.
        if (index > 16) index = 16;

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {
        long remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}