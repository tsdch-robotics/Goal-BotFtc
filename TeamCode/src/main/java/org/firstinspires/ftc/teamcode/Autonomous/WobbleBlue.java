/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public abstract class WobbleBlue extends LinearOpMode {
    DcMotor FrontLeftMotor;
    DcMotor FrontRightMotor;
    DcMotor BackLeftMotor;
    DcMotor BackRightMotor;
    DcMotor ArmMotor;
    int milliseconds = 0;
    double LeftPower = 0;
    double RightPower = 0;
    int call = 0;
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        static final int FOUR_RING_THRESHOLD = 150;
        static final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            int call = 0;
            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
                call = 4;
                WobbleBlue_A wobbleBlue_a = new WobbleBlue_A();
                WobbleBlue_A.WobbleBlue_A();
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
                call = 1;
            } else {
                position = RingPosition.NONE;
                call = 0;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }

    public void Targetzones() {
        FrontLeftMotor = hardwareMap.dcMotor.get("DriveFrontLeft");
        FrontRightMotor = hardwareMap.dcMotor.get("DriveFrontRight");
        BackLeftMotor = hardwareMap.dcMotor.get("DriveBackLeft");
        BackRightMotor = hardwareMap.dcMotor.get("DriveBackRight");
        ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        BackRightMotor.setDirection(DcMotor.Direction.REVERSE);
        if (call == 0) {
            DriveRobot(3935, .5, .5, .5, .5, 0);//move forward 80.75 in
            sleep(250);//wait for 1 sec
            DriveRobot(650, 0, 0, 0, 0, -1);//move the arm down to horizontal
            sleep(250);//wait for 1 sec
            DriveRobot(260, .5, .5, .5, .5, 0);//move forward 5 in
            sleep(250);//wait for 1 sec
            DriveRobot(650, 0, 0, 0, 0, 1);//move the arm up to vertical
            sleep(250);//wait for 1 sec
            DriveRobot(2900, 0.5, .5, -.5, -0.5, 0);//turn back
            sleep(250);//wait for 1 sec
            DriveRobot(4195, .5, .5, .5, .5, 0);//move forward 80.75 in
            sleep(250);//wait for 1 sec
            DriveRobot(650, 0, 0, 0, 0, -1);//move the arm down to horizontal
            sleep(250);//wait for 1 sec
            DriveRobot(520, -.5, -.5, -.5, -.5, 0);//move backwards 10 in
            sleep(250);//wait for 1 sec
            DriveRobot(650, 0, 0, 0, 0, 1);//move the arm up to vertical
            sleep(250);//wait for 1 esc
            DriveRobot(3935, -.5, -.5, -.5, -.5, 0);//move backward 80.75 in
            sleep(250);//wait for 1 sec
            DriveRobot(2900, -.5, -.5, .5, .5, 0);//turn back
            sleep(250);//wait for 1 sec
            DriveRobot(650, 0, 0, 0, 0, -1);//move the arm down to horizontal
            sleep(250);//wait for 1 sec
            DriveRobot(260, .5, .5, .5, .5, 0);//move forward 5 in
            DriveRobot(650, 0, 0, 0, 0, 1);//move the arm up to vertical
        }
        if (call == 1) {
            DriveRobot(1182, .5, .5, .5, .5, 0);//move forward 22.75 in
            sleep(250);//wait for 1 sec
            DriveRobot(1350, 0.5, .5, -.5, -0.5, 0);//turn right
            sleep(250);//wait for 0.25 sec
            DriveRobot(1182, .5, .5, .5, .5, 0);//move forward 22.75 in
            sleep(250);//wait for 1 sec
            DriveRobot(1350, -0.5, -.5, .5, 0.5, 0);//turn left
            sleep(250);//wait for 0.25 sec
            DriveRobot(3935, .5, .5, .5, .5, 0);//move forward 80.75 in
            sleep(250);//wait for 1 sec
            DriveRobot(650, 0, 0, 0, 0, -1);//move the arm down to horizontal
            sleep(250);//wait for 0.25 sec
            DriveRobot(260, .5, .5, .5, .5, 0);//move forward 5 in
            sleep(250);//wait for 1 sec
            DriveRobot(650, 0, 0, 0, 0, 1);//move the arm up to vertical
            sleep(250);//wait for 0.25 sec
            DriveRobot(1182, -.5, -.5, -.5, -.5, 0);//move backwards 22.75 in
            sleep(250);//wait for 1 sec
            DriveRobot(600, 0, 0, 0, 0, -1);//move the arm down to horizontal
            sleep(250);//wait for 0.25 sec
            DriveRobot(3935, -.5, -.5, -.5, -.5, 0);//move backwards 80.75 in
            sleep(250);//wait for 1 sec
            DriveRobot(600, 0, 0, 0, 0, 1);//move the arm up to vertical
            sleep(250);//wait for 0.25 sec
            DriveRobot(5195, .5, .5, .5, .5, 0);//move forward 100 in
            sleep(250);//wait for 1 sec
            DriveRobot(650, 0, 0, 0, 0, -1);//move the arm down to horizontal
            sleep(250);//wait for 0.25 sec
            DriveRobot(260, .5, .5, .5, .5, 0);//move forward 5 in
            sleep(250);//wait for 1 sec
            DriveRobot(650, 0, 0, 0, 0, 1);//move the arm up to vertical
            sleep(250);//wait for 0.25 sec
            DriveRobot(1182, -.5, -.5, -.5, -.5, 0);//move backwards 22.75 in
            sleep(250);//wait for 1 sec
        }
        if (call == 4) {
            DriveRobot(571, -1, 1, 1, -1, 0);//strafe left 11 in
            sleep(250);
            DriveRobot(6117, .5, .5, .5, .5, 0);//move forward 117.75 in
            sleep(250);
            DriveRobot(650, 0, 0, 0, 0, -1);//move the arm down to horizontal
            sleep(250);
            DriveRobot(260, .5, .5, .5, .5, 0);//move forward 5 in
            sleep(250);
            DriveRobot(1753, 1, -1, -1, 1, 0);//strafe right 33.75 in
            sleep(250);
            DriveRobot(6117, -.5, -.5, -.5, -.5, 0);//move backward 117.75 in
            sleep(250);
            DriveRobot(650, 0, 0, 0, 0, 1);//move the arm up to vertical
            sleep(250);
            DriveRobot(1753, -1, 1, 1, -1, 0);//strafe left 33.75 in
            sleep(250);
            DriveRobot(6117, .5, .5, .5, .5, 0);//move forward 117.75 in
            sleep(250);
            DriveRobot(650, 0, 0, 0, 0, -1);//move the arm down to horizontal
            sleep(250);
            DriveRobot(260, .5, .5, .5, .5, 0);//move forward 5 in
            sleep(250);
            DriveRobot(650, 0, 0, 0, 0, 1);//move the arm up to vertical
            sleep(250);
            DriveRobot(1558, -.5, -.5, -.5, -.5, 0);//move backward 30 in
        }
    }

    private void DriveRobot(int milliseconds, double LeftFrontPower, double LeftBackPower, double RightFrontPower, double RightBackPower, double ArmPower) {
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to x power.

        FrontLeftMotor.setPower(LeftFrontPower);
        FrontRightMotor.setPower(RightFrontPower);
        BackLeftMotor.setPower(LeftBackPower);
        BackRightMotor.setPower(RightBackPower);
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