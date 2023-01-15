package org.firstinspires.ftc.teamcode.drive.opmode;

/*
 * Copyright (c) 2021 OpenFTC Team
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous(name="Tomahawk Auton")
public class Auton extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx slide = hardwareMap.get(DcMotorEx.class, "slide");
        Servo claw1 = hardwareMap.get(Servo.class, "claw1");

        //TODO--------------------------------------------------------------------------------------------
        final int smallPolePos = 1094;
        final int mediumPolePos = 1790;
        final int longPolePos = 2500;
        final int stack = 100;


        Pose2d startPose = new Pose2d(-36, -72, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence toHigh = drive.trajectorySequenceBuilder (startPose)
                .addTemporalMarker(0, () -> {
                    slide.setTargetPosition(200);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(.8);
                })
                .strafeLeft(72)
                // (-36, 0)
                .addTemporalMarker(0, () -> {
                    slide.setTargetPosition(longPolePos);
                    slide.setPower(.8);
                })
                .forward(3)
                // (-33, 0)
                .build();

        //Takes robot in the middle of the square right in front of the stack
        TrajectorySequence toFaceStackLn = drive.trajectorySequenceBuilder(toHigh.end())
                .back(3)
                // (-36, 0)
                .addTemporalMarker(0, () -> {
                    slide.setTargetPosition(0);
                    slide.setPower(-.8);
                })
                .strafeRight(12)
                .turn(Math.toRadians(180))
                .forward(24)
//                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)))
                .build();

        //Takes robot right in front of the stack
        TrajectorySequence toConeStack = drive.trajectorySequenceBuilder(toFaceStackLn.end())
                .addTemporalMarker(0, () -> {
                    slide.setTargetPosition(stack);
                    slide.setPower(.8);
                })
                .forward(24+11)
                .build();

        //Basically toFaceStackLn and toConeStack combined into a spline
        TrajectorySequence toStackSpl = drive.trajectorySequenceBuilder (toHigh.end())
                .addTemporalMarker(0, () -> {
                    slide.setTargetPosition(0);
                    slide.setPower(-.8);
                })
                .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
                .build();

        //Takes robot from cone stack to high pole
        TrajectorySequence toHighAgain = drive.trajectorySequenceBuilder (toConeStack.end()) //TODO (ask abraham what the fuck he did)
                .addTemporalMarker(0, () -> {
                    slide.setTargetPosition(200);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(.8);
                })
                .strafeLeft(72)
                // (-36, 0)
                .addTemporalMarker(0, () -> {
                    slide.setTargetPosition(longPolePos);
                    slide.setPower(.8);
                })
                .forward(3)
                // (-33, 0)
                .build();

        //TODO--------------------------------------------------------------------------------------------

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //TODO------------------------------------------------------------------------------
        telemetry.addData("before", "y");
        claw1.setPosition(.7);
        sleep(2000);
        drive.followTrajectorySequence(toHigh);
        claw1.setPosition(.2);
        sleep(2000);
        drive.followTrajectorySequence(toFaceStackLn);
        sleep(10000);
        telemetry.addData("after", "y");
        //TODO------------------------------------------------------------------------------

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {

        } else if (tagOfInterest.id == MIDDLE) {

        } else {

        }


        telemetry.update();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

