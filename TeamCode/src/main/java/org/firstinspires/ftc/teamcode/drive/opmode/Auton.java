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
        DcMotorEx slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

        slide.setDirection(DcMotorEx.Direction.FORWARD);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Servo claw1 = hardwareMap.get(Servo.class, "claw1");

        //TODO--------------------------------------------------------------------------------------------
        final int smallPolePos = 487;
        final int mediumPolePos = 783;
        final int longPolePos = 1048;
        final double clawClose = 0.7;
        final double clawOpen = 0.2;
        final int stack = 100;


        Pose2d startPose = new Pose2d(-36, -68, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence toMedium1 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    slide2.setTargetPosition(200);
                    slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide2.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(-33,-24,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    slide2.setTargetPosition(mediumPolePos);
                    slide2.setPower(1);
                })
                .build();

        TrajectorySequence toStack1 = drive.trajectorySequenceBuilder(toMedium1.end())
                .addTemporalMarker(0, () -> {
                    slide2.setTargetPosition(184);
                    slide2.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(-63,-12,Math.toRadians(180) -1e-6))
                .build();

        TrajectorySequence toMedium2 = drive.trajectorySequenceBuilder(toStack1.end())
                .addTemporalMarker(0, () -> {
                    slide2.setTargetPosition(mediumPolePos);
                    slide2.setPower(1);
                })
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-22.5,-14,Math.toRadians(-90) +1e-6))
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(toMedium2.end())
                .addTemporalMarker(0, () -> {
                    claw1.setPosition(clawClose);
                    slide2.setTargetPosition(160);
                    slide2.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(-65,-12,Math.toRadians(180) -1e-6))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    claw1.setPosition(clawOpen);
                })
                .build();

        TrajectorySequence toMedium3 = drive.trajectorySequenceBuilder(toStack2.end())
                .addTemporalMarker(0, () -> {
                    slide2.setTargetPosition(mediumPolePos);
                    slide2.setPower(1);
                })
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-23,-16,Math.toRadians(-90) +1e-6))
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(toMedium3.end())
                .addTemporalMarker(0, () -> {
                    slide2.setTargetPosition(0);
                    slide2.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(-60,-12,Math.toRadians(-90)))
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(toMedium3.end())
                .addTemporalMarker(0, () -> {
                    slide2.setTargetPosition(0);
                    slide2.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(-90)))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(toMedium3.end())
                .addTemporalMarker(0, () -> {
                    slide2.setTargetPosition(0);
                    slide2.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(-90)))
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

        claw1.setPosition(clawClose);
        sleep(1000);
        drive.followTrajectorySequence(toMedium1);
        claw1.setPosition(clawOpen);
        drive.followTrajectorySequence(toStack1);
        claw1.setPosition(clawClose);
        sleep(300);
        drive.followTrajectorySequence(toMedium2);
        claw1.setPosition(clawOpen);
        drive.followTrajectorySequence(toStack2);
        claw1.setPosition(clawClose);
        sleep(300);
        drive.followTrajectorySequence(toMedium3);
        claw1.setPosition(clawOpen);
        sleep(500);

        telemetry.addData("after", "y");
        //TODO------------------------------------------------------------------------------

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            claw1.setPosition(clawClose);
            drive.followTrajectorySequence(leftPark);
        } else if (tagOfInterest.id == MIDDLE) {
            claw1.setPosition(clawClose);
            drive.followTrajectorySequence(middlePark);
        } else {
            claw1.setPosition(clawClose);
            drive.followTrajectorySequence(rightPark);
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

